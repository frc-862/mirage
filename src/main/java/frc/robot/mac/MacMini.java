package frc.robot.mac;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;

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
        // If someone deploys new Mac code but forgets to deploy new RIO code
        // (or vice versa), the version mismatch will cause packets to be
        // rejected instead of silently misinterpreting the data.
        static final byte PROTOCOL_VERSION = 1;

        // PACKET_SIZE: Total bytes in one packet.
        //   4 (magic) + 1 (version) + 4 (sequence) + 5×8 (doubles) = 49
        static final int PACKET_SIZE = 49;

        // The network tables instance that will connect to the local photonvision server
        NetworkTableInstance photonNT = NetworkTableInstance.create();

        // Cameras
        CameraInfo[] cameras;

        // Socket to send data
        DatagramSocket socket;

        // The resolved IP address of the RoboRIO. We resolve this ONCE in
        // the constructor instead of calling InetAddress.getByName("10.8.62.2")
        // on every packet send. Even though Java may cache DNS lookups,
        // there's no reason to do string parsing and lookup ~1000 times/sec
        // when the address never changes.
        InetAddress rioAddress;

        // Sequence counter — incremented for every packet we send.
        // The receiver can use this to detect if packets were dropped or
        // arrived out of order. Starts at 0 and wraps around at Integer.MAX_VALUE.
        private int sequenceNumber = 0;

        // Why does the constructor throw SocketException now?
        // Previously, if the socket failed to create, we caught the exception,
        // logged it, and continued running with socket = null. Then run() would
        // call socket.send() on a null socket, throwing an NPE every iteration
        // forever — filling the log with useless errors and burning CPU.
        //
        // By letting the exception propagate ("throws SocketException"), we
        // force the program to stop immediately with a clear error message.
        // This is called "fail fast" — it's better to crash with a helpful
        // error than to limp along in a broken state.
        public MacMini() throws IOException {
            // Create a new socket to send data to the rio.
            // If this fails, the exception propagates up and stops the program
            // immediately — there's no point continuing without a socket.
            socket = new DatagramSocket();

            // Resolve the RoboRIO's IP address once, up front.
            // Previously, InetAddress.getByName("10.8.62.2") was called inside
            // getBinaryPacket() on every single packet send (~1000/sec). Even
            // though Java caches DNS results, parsing the string and doing the
            // lookup every time is wasteful. Resolving once here is cleaner.
            rioAddress = InetAddress.getByName("10.8.62.2");

            // Connect to our local network tables server
            photonNT.setServer("localhost", 5810);
            photonNT.startClient4("mac-photon-client");

            // Create an empty array of cameras
            cameras = new CameraInfo[VisionConstants.CAMERA_CONSTANTS.length];
            
            // Create the cameras
            for (int i = 0; i < VisionConstants.CAMERA_CONSTANTS.length; i++) {
                log("Creating " + VisionConstants.CAMERA_CONSTANTS[i].name() + " info");
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
                        new PhotonPoseEstimator(fieldLayout,VisionConstants.CAMERA_CONSTANTS[i].offset());
                    
                // Create the camera using the name from our constant
                PhotonCamera camera = new PhotonCamera(photonNT, VisionConstants.CAMERA_CONSTANTS[i].name());

                // Create the camera
                cameras[i] = new CameraInfo(camera, poseEstimator);
            }
        }

        public void run() {
            while (true) {
                VisionInfo info = getEstimatedPose();

                // IMPORTANT: getEstimatedPose() can return null (not just a
                // VisionInfo with null fields). We must check for null FIRST,
                // otherwise calling info.pose() on a null reference throws a
                // NullPointerException and crashes the entire vision processor.
                if (info != null && info.pose != null && info.result != null) {
                    Pose2d poseToPublish = info.pose().estimatedPose.toPose2d();
                    double ambiguity = info.result().getBestTarget().poseAmbiguity;
                    double timestamp = info.result().getTimestampSeconds();

                    try {
                        DatagramPacket packet = getBinaryPacket(poseToPublish, ambiguity, timestamp);
                        socket.send(packet);
                         
                    } catch (IOException e) {
                        log("Failed to send packet: " + e);
                    }
                }
                
                try {
                    Thread.sleep(1);
                } catch (InterruptedException e) {
                    log("Error sleeping: " + e.getMessage());
                }
            }
        }

        private VisionInfo getEstimatedPose() {
            if (cameras == null || cameras.length == 0) {
                log("No cameras configured");
                return null;
            }

            try {
                VisionInfo[] poses = new VisionInfo[VisionConstants.CAMERA_CONSTANTS.length];
                
                for (int i = 0; i < VisionConstants.CAMERA_CONSTANTS.length; i++) {
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
            System.out.println("[PHOTON VISION]" + message);
        }

        private DatagramPacket getBinaryPacket(Pose2d pose, double ambiguity, double timestamp) {
            ByteBuffer buffer = ByteBuffer.allocate(PACKET_SIZE);

            // --- Header (9 bytes) ---
            // These fields let the receiver verify that this packet is really
            // from our vision code and not random network noise.
            buffer.putInt(MAGIC_NUMBER);        // 4 bytes: identifies this as our packet
            buffer.put(PROTOCOL_VERSION);       // 1 byte:  catches version mismatches
            buffer.putInt(sequenceNumber++);    // 4 bytes: lets receiver detect dropped packets

            // --- Payload (40 bytes) ---
            buffer.putDouble(pose.getX());
            buffer.putDouble(pose.getY());
            buffer.putDouble(pose.getRotation().getRadians());
            buffer.putDouble(ambiguity);
            buffer.putDouble(timestamp);

            byte[] data = buffer.array();
            // Use the pre-resolved rioAddress instead of calling
            // InetAddress.getByName() every time. See constructor.
            return new DatagramPacket(data, data.length, rioAddress, 12345);
        }

        @Override
        public void close() throws Exception {
            // Close our connections when the program closes
            socket.close();
            photonNT.close();
        }
    }