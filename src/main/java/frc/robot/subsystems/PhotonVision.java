// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.nio.ByteBuffer;
import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.util.shuffleboard.LightningShuffleboard;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

public class PhotonVision extends SubsystemBase implements AutoCloseable {
    // Records to store specific groups of data
    private record VisionInfo(double timestamp, double ambiguity, Pose2d pose) {};

    // The drivetrain to add vision measurments
    Swerve drivetrain;

    // An atomic refrence to store our newley recieved pose, thread safe
    AtomicReference<VisionInfo> pose;

    // Stores the time offset for the difference in time between mac and rio
    double macTimeOffset = 0;

    // A datagram socket which we connect to to recieve packets from the mac
    DatagramSocket socket;

    // Thread to run code recieve vision data from the socket
    Thread receiveThread;

    //Thread to check if the mac mini is online
    Thread reachableThread;

    //Shows whether the mac mini is connected or not
    volatile boolean macMiniIsConnected;

    private BooleanLogEntry macConnectedLog;
    private DoubleLogEntry macPingLog;

    private AtomicReference<Time> macMiniPing;

    private static final String MAC_MINI_IP = "10.8.62.11";
    private static final int VISION_PORT = 12345;

    /** Creates a new PhotonVision.
     * 
     * @param drivetrain The main drivetrain on the robot
     */
    public PhotonVision(Swerve drivetrain) {
        if (Robot.isSimulation()) {
            PhotonCamera.setVersionCheckEnabled(false);
        }

        this.drivetrain = drivetrain;
        pose = new AtomicReference<>(null);
        macMiniPing = new AtomicReference<>(Seconds.of(0));

        try {
            // Bind to the port
            socket = new DatagramSocket(VISION_PORT); 
        } catch (SocketException e) {
            log("*** ERROR CREATING DATAGRAM SOCKET ***" + e);
        }

        // Start a separate thread to receive packets
        receiveThread = new Thread(() -> {
            // Run while the thread is still valid
            while (!Thread.currentThread().isInterrupted()) {
                try {
                    // Create a new packet to fill with recieved data
                    byte[] receiveData = new byte[40];
                    var receivePacket = new DatagramPacket(receiveData, receiveData.length, VISION_PORT);
                    
                    if (socket != null) {
                        socket.receive(receivePacket);
                    } else {
                        break;
                    }
                    
                    // Fresh vision data :)
                    VisionInfo data = parseBinaryPacket(receivePacket);

                    // Store data atomically
                    pose.set(data);
                    
                } catch (IllegalArgumentException e) {
                    log("Thread Error: " + e.getMessage());
                } catch (IOException e) {
                    log("Error recieving packet");
                }
            }
        });

        receiveThread.setDaemon(true);
        receiveThread.start();

        reachableThread = new Thread(() -> {
            while (!Thread.currentThread().isInterrupted()) {
                try {
                    InetAddress macMiniAddress = InetAddress.getByName(MAC_MINI_IP);
                    double startTime = Utils.getCurrentTimeSeconds();
                    macMiniIsConnected = macMiniAddress.isReachable(1000); //timeout in ms
                    macMiniPing.set(Seconds.of(Utils.getCurrentTimeSeconds() - startTime));
                } catch (IOException e) {
                    macMiniIsConnected = false;
                    macMiniPing.set(Seconds.of(0));
                }

                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    log("Error sleeping: " + e.getMessage());
                }
            }
        });

        reachableThread.setDaemon(true);
        reachableThread.start();

        initLogging();
    }

    private void initLogging() {
        DataLog log = DataLogManager.getLog();

        macConnectedLog = new BooleanLogEntry(log, "/Vision/isMacConnected");
        macPingLog = new DoubleLogEntry(log, "/Vision/macMiniPing");
    }
    
    @Override
    public void periodic() {
        LightningShuffleboard.setDouble("Vision", "robot_time", Utils.getCurrentTimeSeconds());
        LightningShuffleboard.setBool("Vision", "is Mac Connected", macMiniIsConnected);
        LightningShuffleboard.setDouble("Vision", "Mac Mini Ping", macMiniPing.get().in(Milliseconds));

        VisionInfo updatedPose = pose.getAndSet(null);
        if (updatedPose != null && updatedPose.pose != null && updatedPose.ambiguity < 1 && updatedPose.timestamp > 0) {
            // If the time offset has not been set yet, calculate it
            if (macTimeOffset == 0) {
                macTimeOffset = Utils.getCurrentTimeSeconds() - updatedPose.timestamp;
            }

            // Calculate the standard deviation to use-- small multiplier so not too low
            double trust = updatedPose.ambiguity() * 1.2;

            LightningShuffleboard.setPose2d("Vision", "updated pose", updatedPose.pose);
            LightningShuffleboard.setDouble("Vision", "mac time offset", macTimeOffset);
            
            // Adds our estimated pose from vision to our drivetrain's pose, fuses with odometry
            drivetrain.addVisionMeasurement(
                updatedPose.pose(), 
                updatedPose.timestamp + macTimeOffset, 
                VecBuilder.fill(trust, trust, trust)
            );
        }     
        updateLogging();
    }
    
    private void updateLogging() {
        macConnectedLog.append(macMiniIsConnected);
        macPingLog.append(macMiniPing.get().in(Milliseconds));
    }

    /**
     * Unpacks a datagram packet into the Unpacked Data object
     * 
     * @param packet The packet to unpack
     * @return Unpacked Data, Pose, ambiguity, timestamp, and a counter
     */
    private VisionInfo parseBinaryPacket(DatagramPacket packet) {
        byte[] data = packet.getData();
        
        // Safety check for length
        if (packet.getLength() < 40) {
            throw new IllegalArgumentException("Packet too small");
        }

        // Wrap the data in a ByteBuffer
        ByteBuffer buffer = ByteBuffer.wrap(data, 0, 40);

        // Read doubles in the same order they were packed
        double x = buffer.getDouble();
        double y = buffer.getDouble();
        double rotRadians = buffer.getDouble();
        double ambiguity = buffer.getDouble();
        double timestamp = buffer.getDouble();

        // Construct a new pose using the unpacked data
        Pose2d newPose = new Pose2d(x, y, new Rotation2d(rotRadians));
        
        return new VisionInfo(timestamp, ambiguity, newPose);
    }

    public boolean getMacMiniConnection() {
        return macMiniIsConnected;
    }

    // Logs a message with the [PHOTON VISION] tag in front 
    private void log(String message) {
        DataLogManager.log("[PHOTON VISION] " + message);
    }

    @Override
    public void close() throws Exception {
        if (socket != null) {
            socket.close();
        }

        reachableThread.interrupt();
        receiveThread.interrupt();
    }
}
