package frc.robot.mac;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.constants.RobotMap;
import frc.robot.mac.VisionConstants.CameraConstant;

public class MacMini implements AutoCloseable {
        // Camera info
        private record CameraInfo(PhotonCamera camera, PhotonPoseEstimator poseEstimator) {};
        private record VisionInfo(PhotonPipelineResult result, EstimatedRobotPose pose) {};

        // ===== PACKET PROTOCOL CONSTANTS =====
        // These must match EXACTLY between the sender (here) and the receiver
        // (PhotonVision.java on the RoboRIO). If you change one, change both!
        //
        // MAGIC_NUMBER: A unique identifier at the start of every packet.
        // Think of it like a secret handshake — if a random UDP packet arrives
        // on our port (from another team's code, a network tool, etc.), it
        // almost certainly won't start with 0x00000862. The receiver checks
        // this first and throws away anything that doesn't match.
        // We use 0x862 = team 862 in hex, padded to 4 bytes.
        static final int MAGIC_NUMBER = 0x00000862;

        // PROTOCOL_VERSION: Incremented whenever the packet format changes.
        // Bumped from 1 to 2 when we added the has_pose flag byte.
        // If someone deploys new Mac code but forgets to deploy new RIO code
        // (or vice versa), the version mismatch will cause packets to be
        // rejected instead of silently misinterpreting the data.
        static final byte PROTOCOL_VERSION = 2;

        // PACKET_SIZE: Total bytes in one packet.
        //   4 (magic) + 1 (version) + 4 (sequence) + 1 (has_pose) + 5×8 (doubles) = 50
        static final int PACKET_SIZE = 50;

        // How often (in ms) the sender thread fires. 10ms = 100Hz.
        // The RIO runs periodic() at 50Hz, so at 100Hz we guarantee at
        // least 1-2 fresh packets per RIO cycle, while the heartbeat
        // lets the RIO know we're alive even when no tags are visible.
        private static final long SEND_INTERVAL_MS = 10;

        // Which camera constants to use — picked at startup based on which
        // robot this is (main robot vs Oasis have different camera positions).
        CameraConstant[] cameraConstants;

        // The network tables instance that will connect to the local photonvision server
        NetworkTableInstance photonNT = NetworkTableInstance.create();

        // Cameras
        CameraInfo[] cameras;

        // Socket to send data
        DatagramSocket socket;

        // The resolved IP address of the RoboRIO. We resolve this ONCE in
        // the constructor instead of calling InetAddress.getByName("10.8.62.2")
        // on every packet send.
        InetAddress rioAddress;

        // Sequence counter — incremented for every packet we send (pose or heartbeat).
        // The receiver can use this to detect if packets were dropped.
        private int sequenceNumber = 0;

        // ===== PRODUCER/SENDER ARCHITECTURE =====
        // We split the work into two threads:
        //
        //   PRODUCER THREAD: runs continuously, polls cameras, computes poses.
        //     Stores the best pose it's seen since the last send in bestPose.
        //
        //   SENDER THREAD: fires every SEND_INTERVAL_MS (10ms). Grabs the
        //     best pose accumulated since the last send. If there is one,
        //     sends a pose packet. If not, sends a heartbeat packet.
        //
        // WHY: Previously a single loop did both — compute a pose, send it,
        // sleep 15ms. That meant:
        //   1. We only sent when we had a pose (no heartbeat when tags weren't visible)
        //   2. Send timing was coupled to vision processing time
        //   3. We only considered one reading per send, not the best of several
        //
        // Now the producer can run as fast as it wants, accumulating the
        // lowest-ambiguity pose over each 10ms window. The sender always
        // fires on schedule regardless of whether vision found anything.

        // Thread-safe storage for the best pose found since the last send.
        // The producer writes to this; the sender reads and clears it.
        private final AtomicReference<VisionInfo> bestPose = new AtomicReference<>(null);

        // The scheduled executor that runs the sender thread on a fixed 10ms cadence.
        private ScheduledExecutorService sendExecutor;

        // The producer thread that continuously polls cameras for poses.
        private Thread producerThread;

        // Flag to signal that a fatal error occurred and run() should exit.
        // This lets Main.java's retry loop catch the error and restart us.
        private volatile boolean fatalError = false;
        private volatile String fatalErrorMessage = "";

        public MacMini() throws IOException {
            // Create a new socket to send data to the rio.
            // If this fails, the exception propagates up and stops the program
            // immediately — there's no point continuing without a socket.
            socket = new DatagramSocket();

            // Resolve the RoboRIO's IP address once, up front.
            rioAddress = InetAddress.getByName("10.8.62.2");

            // Connect to our local network tables server
            photonNT.setServer("localhost", 5810);
            photonNT.startClient4("mac-photon-client");

            cameraConstants = RobotMap.IS_OASIS ? VisionConstants.OASIS_CAMERA_CONSTANTS : VisionConstants.CAMERA_CONSTANTS;

            // Create an empty array of cameras
            cameras = new CameraInfo[cameraConstants.length];


            // Create the cameras
            for (int i = 0; i < cameraConstants.length; i++) {
                log("Creating " + cameraConstants[i].name() + " info");
                AprilTagFieldLayout fieldLayout;

                try {
                    // Get the path to a custom field from the home directory
                    Path fieldPath = Path.of(
                        System.getProperty("user.home"),
                        "field_layout.json"
                    );

                    fieldLayout = new AprilTagFieldLayout(fieldPath);
                } catch (IOException e) {
                    // Just use the default field if we can't get it
                    log("Can't load field resource-- using default field");
                    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
                }

                // Get the pose estimator
                PhotonPoseEstimator poseEstimator =
                        new PhotonPoseEstimator(fieldLayout, cameraConstants[i].offset());


                // Create the camera using the name from our constant
                PhotonCamera camera = new PhotonCamera(photonNT, cameraConstants[i].name());

                // Create the camera
                cameras[i] = new CameraInfo(camera, poseEstimator);
            }
        }

        /**
         * Starts the producer and sender threads and blocks until a fatal error occurs.
         * When this method returns, the caller (Main.java) should close() and retry.
         */
        public void run() {
            fatalError = false;

            // --- Start the PRODUCER thread ---
            // This thread continuously polls cameras and accumulates the best
            // pose seen since the last send. It uses AtomicReference.accumulateAndGet()
            // to thread-safely keep whichever pose has lower ambiguity.
            producerThread = new Thread(() -> {
                while (!Thread.currentThread().isInterrupted() && !fatalError) {
                    try {
                        VisionInfo info = getEstimatedPose();

                        if (info != null && info.pose != null && info.result != null) {
                            // accumulateAndGet is an atomic read-modify-write operation.
                            // It compares the new pose against whatever is currently stored,
                            // and keeps whichever has lower ambiguity (= higher quality).
                            // This means the sender always gets the BEST pose from the
                            // entire 10ms window, not just whichever happened to be last.
                            bestPose.accumulateAndGet(info, (current, candidate) -> {
                                if (current == null) {
                                    return candidate;
                                }
                                double currentAmb = current.result().getBestTarget().poseAmbiguity;
                                double candidateAmb = candidate.result().getBestTarget().poseAmbiguity;
                                return candidateAmb < currentAmb ? candidate : current;
                            });
                        }

                        // Small sleep to avoid busy-waiting. 5ms means we get ~2
                        // readings per 10ms send window on average.
                        Thread.sleep(5);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    } catch (Exception e) {
                        // Log but don't crash — the producer should keep trying.
                        // Camera disconnects, NetworkTables hiccups, etc. are
                        // transient and will resolve on their own.
                        log("Producer error: " + e.getMessage());
                    }
                }
            });
            producerThread.setDaemon(true);
            producerThread.start();

            // --- Start the SENDER thread ---
            // This runs on a fixed 10ms schedule using ScheduledExecutorService,
            // which is more accurate than Thread.sleep() for periodic tasks.
            // It always sends a packet — either a pose or a heartbeat.
            sendExecutor = Executors.newSingleThreadScheduledExecutor();
            sendExecutor.scheduleAtFixedRate(() -> {
                try {
                    // Grab the best pose accumulated since the last send,
                    // and clear it so the producer starts fresh for the next window.
                    VisionInfo info = bestPose.getAndSet(null);

                    DatagramPacket packet;
                    if (info != null && info.pose != null && info.result != null) {
                        // We have a valid pose — send it with has_pose = 1
                        Pose2d poseToPublish = info.pose().estimatedPose.toPose2d();
                        double ambiguity = info.result().getBestTarget().poseAmbiguity;
                        double timestamp = info.result().getTimestampSeconds();
                        packet = buildPacket(true, poseToPublish, ambiguity, timestamp);
                    } else {
                        // No pose available — send a heartbeat with has_pose = 0.
                        // The RIO will see this and know "comms are good, just
                        // no tags visible right now" instead of "is the Mac dead?"
                        packet = buildPacket(false, null, 0, 0);
                    }

                    socket.send(packet);

                } catch (IOException e) {
                    // If the socket itself is broken (closed, etc.), signal
                    // a fatal error so run() exits and Main.java can restart us.
                    if (socket.isClosed()) {
                        fatalError = true;
                        fatalErrorMessage = "Socket closed: " + e.getMessage();
                    } else {
                        // Transient send failure — log and try again next cycle
                        log("Send failed: " + e.getMessage());
                    }
                }
            }, 0, SEND_INTERVAL_MS, TimeUnit.MILLISECONDS);

            // Block until a fatal error occurs. The producer and sender threads
            // do the real work; we just wait here so Main.java's retry loop
            // knows when to restart us.
            while (!fatalError) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    break;
                }
            }

            log("Fatal error, exiting run(): " + fatalErrorMessage);
        }

        /**
         * Builds a UDP packet with the current protocol format.
         *
         * @param hasPose  true if this packet carries a valid pose, false for heartbeat-only
         * @param pose     the robot pose (ignored if hasPose is false)
         * @param ambiguity the pose ambiguity (ignored if hasPose is false)
         * @param timestamp the vision timestamp in seconds (ignored if hasPose is false)
         * @return a DatagramPacket ready to send
         */
        private DatagramPacket buildPacket(boolean hasPose, Pose2d pose, double ambiguity, double timestamp) {
            ByteBuffer buffer = ByteBuffer.allocate(PACKET_SIZE);
            buffer.order(ByteOrder.BIG_ENDIAN);

            // --- Header (10 bytes) ---
            buffer.putInt(MAGIC_NUMBER);        // 4 bytes: identifies this as our packet
            buffer.put(PROTOCOL_VERSION);       // 1 byte:  catches version mismatches
            buffer.putInt(sequenceNumber++);    // 4 bytes: lets receiver detect dropped packets

            // has_pose flag: 1 = this packet contains a valid pose in the
            // payload. 0 = heartbeat only, payload is zeroed out.
            // This is how the RIO tells the difference between "the Mac is
            // alive but can't see any AprilTags" vs "the Mac is dead."
            buffer.put((byte) (hasPose ? 1 : 0)); // 1 byte

            // --- Payload (40 bytes) ---
            if (hasPose) {
                buffer.putDouble(pose.getX());
                buffer.putDouble(pose.getY());
                buffer.putDouble(pose.getRotation().getRadians());
                buffer.putDouble(ambiguity);
                buffer.putDouble(timestamp);
            } else {
                // Heartbeat: fill payload with zeros. The receiver will
                // ignore these bytes when has_pose == 0.
                buffer.put(new byte[40]);
            }

            byte[] data = buffer.array();
            return new DatagramPacket(data, data.length, rioAddress, 12345);
        }

        private VisionInfo getEstimatedPose() {
            if (cameras == null || cameras.length == 0) {
                log("No cameras configured");
                return null;
            }

            try {
                VisionInfo[] poses = new VisionInfo[cameraConstants.length];

                for (int i = 0; i < cameraConstants.length; i++) {
                    poses[i] = getVisionPose(cameras[i]);
                }

                return getBestPose(poses);
            } catch (Exception e) {
                log("Failed to get pose");
                return null;
            }
        }

        // Gets the latest result from multiple results
        private PhotonPipelineResult getLatestResult(List<PhotonPipelineResult> results) {
            int latestResultIndex = 0;

            for (int i = 0; i < results.size(); i++) {
                if (results.get(i).getTimestampSeconds() > results.get(latestResultIndex).getTimestampSeconds()) {
                    latestResultIndex = i;
                }
            }

            PhotonPipelineResult latestResult = results.get(latestResultIndex);

            return latestResult;
        }

        // This will get the best pose
        private VisionInfo getBestPose(VisionInfo[] visionInfos) {
            VisionInfo bestPose = null;

            for (VisionInfo info : visionInfos) {
                if (info == null) {
                    continue;
                }

                if (bestPose == null) {
                    bestPose = info;
                    continue;
                }

                if (bestPose.result().getBestTarget().poseAmbiguity > info.result().getBestTarget().poseAmbiguity) {
                    bestPose = info;
                }
            }
            return bestPose == null ? new VisionInfo(null, null) : bestPose;
        }

        private VisionInfo getVisionPose(CameraInfo cameraInfo) {
            PhotonCamera camera = cameraInfo.camera;

            List<PhotonPipelineResult> results = camera.getAllUnreadResults();

            // If theres no results just skip this iteration
            if (results.isEmpty()) {
                return null;
            }

            // Get the latest result of all thme
            PhotonPipelineResult latestResult = getLatestResult(results);

            // Filter out the targets
            List<PhotonTrackedTarget> filteredTargets = new ArrayList<>(latestResult.getTargets());
            filteredTargets.removeIf((tag) -> VisionConstants.TAG_IGNORE_LIST.contains((short) tag.getFiducialId()));

            // Scrap it if the new result has no target
            if (filteredTargets.isEmpty()) {
                return null;
            }

            PhotonPipelineResult useableResult = latestResult;

            // Create a new result to use -- Using the same metadata as the original latest result
            if (!latestResult.getTargets().stream().allMatch((target) -> filteredTargets.contains(target))) {
                useableResult = new PhotonPipelineResult(
                    latestResult.metadata,
                    filteredTargets,
                    Optional.empty()
                );
            }

            // Check pose ambiguity and distance on the FILTERED result, not
            // the raw one. Why? If we check latestResult (before filtering),
            // the "best target" might be a tag we're ignoring. That ignored
            // tag could have high ambiguity, causing us to reject the entire
            // result — even though the REMAINING tags (after filtering) have
            // perfectly good ambiguity.
            //
            // Example: Tag 7 (ignored) has ambiguity 0.8, Tag 3 has ambiguity 0.1.
            //   Old code: checks Tag 7's 0.8 > 1.0? No, but if it were 1.1, we'd
            //             reject the result even though Tag 3 is excellent.
            //   New code: checks useableResult's best target (Tag 3, ambiguity 0.1).
            boolean highPoseAmbiguity = useableResult.getBestTarget().getPoseAmbiguity() > VisionConstants.POSE_AMBIGUITY_TOLERANCE;

            // Check distance on the filtered result too, for the same reason.
            double bestDistance = useableResult.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();
            boolean highDistance = bestDistance > VisionConstants.TAG_DISTANCE_TOLERANCE;

            // If we have high ambiguity or high distance then just return back a null/"empty" value
            if (highPoseAmbiguity || highDistance) {
                return null;
            }

            // Get the estimated position
            Optional<EstimatedRobotPose> poseOpt = cameraInfo.poseEstimator().estimateCoprocMultiTagPose(useableResult);

            // If the estimated position is there run this code
            if (poseOpt.isPresent()) {
                // The pose
                EstimatedRobotPose pose = poseOpt.get();

                // Add the vision measurment
                return new VisionInfo(useableResult, pose);
            } else {
                // Get the estimated position
                poseOpt = cameraInfo.poseEstimator().estimateLowestAmbiguityPose(useableResult);

                if (poseOpt.isPresent()) {
                    // The pose
                    EstimatedRobotPose pose = poseOpt.get();

                    // Add the vision measurment
                    return new VisionInfo(useableResult, pose);
                }
            }

            // We have no pose if were here
            log("No pose");
            return null;
        }

        private void log(String message) {
            System.out.println("[PHOTON VISION] " + message);
        }

        @Override
        public void close() throws Exception {
            // Stop the sender thread first (it uses the socket)
            if (sendExecutor != null) {
                sendExecutor.shutdownNow();
            }

            // Stop the producer thread
            if (producerThread != null) {
                producerThread.interrupt();
            }

            // Close our connections when the program closes
            if (socket != null && !socket.isClosed()) {
                socket.close();
            }
            photonNT.close();
        }
    }
