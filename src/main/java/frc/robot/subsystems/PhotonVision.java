// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.util.concurrent.atomic.AtomicReference;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.shuffleboard.LightningShuffleboard;

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

    /** Creates a new PhotonVision.
     * 
     * @param drivetrain The main drivetrain on the robot
     */
    public PhotonVision(Swerve drivetrain) {
        this.drivetrain = drivetrain;
        pose = new AtomicReference<>(null);

        try {
            // Bind to the port
            socket = new DatagramSocket(12345,InetAddress.getByName("10.8.62.2")); 
        } catch (SocketException e) {
            log("*** ERROR CREATING DATAGRAM SOCKET ***" + e);
        } catch (UnknownHostException e) {
            log("***ERROR CREATING SOCKET -- HOST DOES NOT EXIST***");
        }

        // Start a separate thread to receive packets
        receiveThread = new Thread(() -> {
            // Run while the thread is still valid
            while (!Thread.currentThread().isInterrupted()) {
                try {
                    // Create a new packet to fill with recieved data
                    byte[] receiveData = new byte[40];
                    var receivePacket = new DatagramPacket(receiveData, receiveData.length, InetAddress.getByName("10.8.62.2"), 12345);
                    socket.receive(receivePacket); // Blocks this thread, not the robot
                    
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

        receiveThread.start();
    }
    
    @Override
    public void periodic() {
        LightningShuffleboard.setDouble("Vision", "robot_time", Utils.getCurrentTimeSeconds());

        if (pose.get() != null && pose.get().pose != null && pose.get().ambiguity < 1 && pose.get().timestamp > 0) {
            // Take the value from the new post and then set it to null
            VisionInfo updatedPose = pose.getAndSet(null);

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

    // Logs a message with the [PHOTON VISION] tag in front 
    private void log(String message) {
        DataLogManager.log("[PHOTON VISION] " + message);
    }

    @Override
    public void close() throws Exception {
        socket.close();
    }
}
