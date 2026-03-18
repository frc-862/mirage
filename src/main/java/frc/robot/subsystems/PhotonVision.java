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
    // Records to store specific groups of data.
    // receiveTimeRio: the RIO's clock time at the moment the UDP packet arrived.
    // We capture this in the receive thread (not in periodic()) so that the
    // scheduling delay between packet arrival and periodic() execution doesn't
    // get baked into our time offset calculation. See fix #4 in NETWORK_FINDINGS.md.
    private record VisionInfo(double timestamp, double ambiguity, Pose2d pose, double receiveTimeRio) {};

    // ===== PACKET PROTOCOL CONSTANTS =====
    // These MUST match the values in MacMini.java. If you change the packet
    // format, update both files and bump PROTOCOL_VERSION.
    private static final int MAGIC_NUMBER = 0x00000862;
    private static final byte PROTOCOL_VERSION = 1;
    private static final int PACKET_SIZE = 49;

    // The drivetrain to add vision measurments
    Swerve drivetrain;

    // An atomic refrence to store our newley recieved pose, thread safe
    AtomicReference<VisionInfo> pose;

    // Stores the time offset for the difference in time between mac and rio.
    //
    // WHY THIS EXISTS:
    // The Mac Mini and the RoboRIO have different system clocks. When the Mac
    // sends a vision timestamp, we need to convert it to the RIO's time domain
    // so the Kalman filter can properly account for how much the robot moved
    // between when the image was captured and when we process it.
    //
    // macTimeOffset = (RIO time) - (Mac time) for the same real-world moment.
    // To convert any Mac timestamp to RIO time: rioTime = macTime + macTimeOffset
    //
    // OLD BEHAVIOR: computed once on the first packet. Whatever UDP latency
    // that packet had was permanently baked in.
    //
    // NEW BEHAVIOR: uses an Exponential Moving Average (EMA) to continuously
    // refine the offset. This averages out random UDP jitter while still
    // tracking any slow clock drift between the two computers.
    double macTimeOffset = 0;

    // EMA smoothing factor for macTimeOffset.
    //   Lower  (e.g., 0.01) = more stable, slower to adapt to clock drift
    //   Higher (e.g., 0.10) = noisier, faster to adapt
    // 0.02 means each new sample contributes 2% and the history contributes 98%.
    private static final double TIME_OFFSET_ALPHA = 0.02;

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

    // Minimum standard deviation (meters) for vision measurements.
    // Prevents the Kalman filter from placing infinite confidence in a
    // single vision reading when ambiguity is near zero. A value of 0.3
    // means "even a perfect multi-tag solve can be off by ~30 cm."
    // Tune this: lower = trust vision more, higher = trust odometry more.
    private static final double MIN_VISION_STD_DEV = 0.3;

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
                    // Create a buffer large enough for our packet (49 bytes).
                    // Must match PACKET_SIZE from the sender.
                    byte[] receiveData = new byte[PACKET_SIZE];
                    var receivePacket = new DatagramPacket(receiveData, receiveData.length);
                    
                    if (socket != null) {
                        socket.receive(receivePacket);
                    } else {
                        break;
                    }

                    // Capture the RIO's clock time RIGHT NOW, at the moment
                    // the packet arrived. This is important for the time offset
                    // calculation in periodic(). If we waited until periodic()
                    // to read the clock, up to 20ms of scheduling delay would
                    // be incorrectly included in the offset.
                    double receiveTimeRio = Utils.getCurrentTimeSeconds();

                    // Fresh vision data :)
                    VisionInfo data = parseBinaryPacket(receivePacket, receiveTimeRio);

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
                    Thread.currentThread().interrupt();
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
            // Compute the instantaneous time offset for THIS packet.
            // instantOffset = how far apart the two clocks were when the
            // packet arrived, including any UDP latency on this specific packet.
            //
            // We use receiveTimeRio (captured in the receive thread the
            // instant the packet arrived) instead of "now" (Utils.getCurrentTimeSeconds()).
            // Why? periodic() runs at 50Hz, so there can be up to 20ms between
            // when the packet arrived and when this line runs. That 20ms would
            // be incorrectly added to every offset measurement, making all
            // vision timestamps systematically late.
            double instantOffset = updatedPose.receiveTimeRio - updatedPose.timestamp;

            if (macTimeOffset == 0) {
                // First packet: use it directly as our initial estimate.
                macTimeOffset = instantOffset;
            } else {
                // Subsequent packets: blend the new measurement into our
                // running estimate using an Exponential Moving Average.
                //
                // EMA formula: new_avg = old_avg * (1 - alpha) + new_sample * alpha
                //
                // This smooths out random UDP jitter (some packets arrive
                // fast, some slow) while still adapting if the two clocks
                // slowly drift apart over the course of a match.
                macTimeOffset = macTimeOffset * (1.0 - TIME_OFFSET_ALPHA)
                              + instantOffset * TIME_OFFSET_ALPHA;
            }

            // The "trust" value is the standard deviation (in meters or radians)
            // we tell the Kalman filter. A SMALLER number = MORE confidence in vision.
            //
            // Problem: when ambiguity is near 0 (e.g. a clean multi-tag solve),
            // trust was also near 0, which tells the Kalman filter "this
            // measurement is perfect — ignore odometry entirely." Even tiny
            // vision errors then cause the pose estimate to jump, which makes
            // the turret chase phantom movements.
            //
            // The fix: clamp trust to a minimum of MIN_VISION_STD_DEV so we
            // always blend vision with odometry rather than blindly trusting it.
            double xyTrust = Math.max(updatedPose.ambiguity() * 1.2, MIN_VISION_STD_DEV);

            // WHY rotation trust is tighter (smaller = more confident):
            //   AprilTag detection constrains rotation very well — the tag's
            //   corners give strong angular information even at distance.
            //   Translation, on the other hand, depends on accurately knowing
            //   the tag's range, which gets noisier with distance.
            //
            //   By trusting the rotation more, the Kalman filter lets vision
            //   correct our heading quickly (important for turret aiming) while
            //   being more conservative about x/y jumps (which cause the turret
            //   to chase phantom lateral movements).
            //
            //   The 0.5 multiplier means rotation trust is twice as tight as
            //   translation trust. Tune this if the heading seems jumpy.
            double rotTrust = xyTrust * 0.5;

            LightningShuffleboard.setPose2d("Vision", "updated pose", updatedPose.pose);
            LightningShuffleboard.setDouble("Vision", "mac time offset", macTimeOffset);

            // Adds our estimated pose from vision to our drivetrain's pose, fuses with odometry.
            // The std dev vector is [x, y, theta] — x and y in meters, theta in radians.
            drivetrain.addVisionMeasurement(
                updatedPose.pose(),
                updatedPose.timestamp + macTimeOffset,
                VecBuilder.fill(xyTrust, xyTrust, rotTrust)
            );
        }     
        updateLogging();
    }
    
    private void updateLogging() {
        macConnectedLog.append(macMiniIsConnected);
        macPingLog.append(macMiniPing.get().in(Milliseconds));
    }

    /**
     * Unpacks a datagram packet into a VisionInfo object.
     *
     * Validates the magic number and protocol version before parsing.
     * This prevents random network traffic from being misinterpreted as
     * vision data and corrupting the Kalman filter.
     *
     * @param packet The packet to unpack
     * @return VisionInfo with pose, ambiguity, and timestamp
     * @throws IllegalArgumentException if the packet is malformed or from an unknown source
     */
    private VisionInfo parseBinaryPacket(DatagramPacket packet, double receiveTimeRio) {
        byte[] data = packet.getData();

        // Safety check: make sure we got a full packet. If someone sends
        // a tiny packet (e.g., a network probe), we reject it here instead
        // of getting a confusing BufferUnderflowException later.
        if (packet.getLength() < PACKET_SIZE) {
            throw new IllegalArgumentException(
                "Packet too small: expected " + PACKET_SIZE + " bytes, got " + packet.getLength());
        }

        ByteBuffer buffer = ByteBuffer.wrap(data, 0, PACKET_SIZE);

        // --- Validate header ---
        // Read the magic number and check it matches. This is our first
        // line of defense against stray packets. Any UDP traffic that
        // doesn't start with 0x00000862 is immediately rejected.
        int magic = buffer.getInt();
        if (magic != MAGIC_NUMBER) {
            throw new IllegalArgumentException(
                "Bad magic number: expected 0x" + Integer.toHexString(MAGIC_NUMBER)
                + ", got 0x" + Integer.toHexString(magic)
                + ". This packet is not from our vision processor.");
        }

        // Check protocol version. If someone deploys new Mac code but old
        // RIO code (or vice versa), this catches it immediately instead of
        // silently misinterpreting the packet fields.
        byte version = buffer.get();
        if (version != PROTOCOL_VERSION) {
            throw new IllegalArgumentException(
                "Protocol version mismatch: expected " + PROTOCOL_VERSION
                + ", got " + version
                + ". Make sure Mac and RIO code are both up to date.");
        }

        // Read the sequence number. We log it but don't reject out-of-order
        // packets — with UDP, occasional reordering is normal and the data
        // is still valid. The sequence number is useful for debugging: if
        // you see big jumps, packets are being dropped on the network.
        int seq = buffer.getInt();

        // --- Parse payload (same order as MacMini.getBinaryPacket) ---
        double x = buffer.getDouble();
        double y = buffer.getDouble();
        double rotRadians = buffer.getDouble();
        double ambiguity = buffer.getDouble();
        double timestamp = buffer.getDouble();

        Pose2d newPose = new Pose2d(x, y, new Rotation2d(rotRadians));

        return new VisionInfo(timestamp, ambiguity, newPose, receiveTimeRio);
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
