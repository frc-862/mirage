package frc.robot.mac;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.nio.file.Path;
import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.format.DateTimeFormatter;
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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.DataLogBackgroundWriter;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.IntegerArrayLogEntry;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.mac.VisionConstants.CameraConstant;

public class MacMini implements AutoCloseable {
        // Camera info
        private record CameraLogEntry(StructLogEntry<Pose2d> robotPoseEntry, IntegerArrayLogEntry tagNumEntries, StructArrayLogEntry<Pose3d> tagPoseEntries, DoubleArrayLogEntry tagDistanceEntries, DoubleArrayLogEntry tagAmbiguityEntries) {};
        private record CameraInfo(PhotonCamera camera, PhotonPoseEstimator poseEstimator, CameraLogEntry logEntry) {}; 
        private record VisionInfo(PhotonPipelineResult result, EstimatedRobotPose pose) {};

        CameraConstant[] cameraConstants;

        // The network tables instance that will connect to the local photonvision server
        NetworkTableInstance photonNT = NetworkTableInstance.create();

        // Cameras
        CameraInfo[] cameras;

        // Best Pose
        StructLogEntry<Pose2d> bestPoseLogEntry;

        // Socket to send data
        DatagramSocket socket;

        // poses from each camera
        private VisionInfo[] poses;

        // data logging
        private DataLog log;
        private long startTime;
        private DoubleLogEntry multiTagAmbiguity;

        public MacMini() {
            startTime = System.nanoTime();

            try {
                // Create a new socket to send data to the rio
                socket = new DatagramSocket();
            } catch (SocketException e) {
                log("*** ERROR CREATING SOCKET ***");
            }

            // Connect to our local network tables server
            photonNT.setServer("localhost", 5810);
            photonNT.startClient4("mac-photon-client");

            cameraConstants = VisionConstants.IS_OASIS ? VisionConstants.OASIS_CAMERA_CONSTANTS : VisionConstants.CAMERA_CONSTANTS;

            // Create an empty array of cameras
            cameras = new CameraInfo[cameraConstants.length];

            // poses from each camera
            poses = new VisionInfo[cameraConstants.length];

            // Create a log file with the current date and time
            DateTimeFormatter timeFormatter = DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss").withZone(ZoneId.of("UTC"));
            LocalDateTime now = LocalDateTime.now(ZoneId.of("UTC"));
            String logFile = "FRC_MacMini_" + timeFormatter.format(now) + ".wpilog";
            log = new DataLogBackgroundWriter(VisionConstants.LOG_PATH, logFile, 0.25);

            bestPoseLogEntry = StructLogEntry.create(log, "bestPose", Pose2d.struct, getMicroSeconds());
            
            // Create the cameras
            for (int i = 0; i < cameraConstants.length; i++) {
                log("Creating " + cameraConstants[i].name() + " info");
                AprilTagFieldLayout fieldLayout;

                try {
                    // Get the path to a custom field from the home directory
                    Path fieldPath = Path.of(
                        System.getProperty("user.home"),
                        "NONE.json"
                    );

                    fieldLayout = new AprilTagFieldLayout(fieldPath);
                } catch (IOException e) {
                    // Just use the default field if we can't get it
                    log("Can't load field resource-- using default field");
                    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
                }

                // Get the pose estimator
                PhotonPoseEstimator poseEstimator =
                        new PhotonPoseEstimator(fieldLayout,cameraConstants[i].offset());
                    
                // Create the camera using the name from our constant
                String cameraName = cameraConstants[i].name();
                PhotonCamera camera = new PhotonCamera(photonNT, cameraName);

                long time = getMicroSeconds();
                CameraLogEntry logEntry = new CameraLogEntry(
                    StructLogEntry.create(log, cameraName + "/robotPose", Pose2d.struct, time), 
                    new IntegerArrayLogEntry(log, cameraName + "/tagNum", time), 
                    StructArrayLogEntry.create(log, cameraName + "/tagPose", Pose3d.struct, time), 
                    new DoubleArrayLogEntry(log, cameraName + "/tagDistance", time), 
                    new DoubleArrayLogEntry(log, cameraName + "/tagAmbiguity", time)
                );

                // TEMP
                multiTagAmbiguity = new DoubleLogEntry(log, "/multiTagAmbiguity");

                // Create the camera
                cameras[i] = new CameraInfo(camera, poseEstimator, logEntry);
            }

        }


        public void run() {
            while (true) {
                VisionInfo info = getEstimatedPose();

                if (info.pose != null && info.result != null) {
                    Pose2d poseToPublish = info.pose().estimatedPose.toPose2d();
                    double ambiguity = info.result().getBestTarget().poseAmbiguity;

                    if (!info.result().getMultiTagResult().isEmpty()) {
                        ambiguity = info.result().getMultiTagResult().get().estimatedPose.ambiguity;
                        multiTagAmbiguity.append(ambiguity);
                    }


                    double timestamp = info.result().getTimestampSeconds();

                    try {
                        DatagramPacket packet = getBinaryPacket(poseToPublish, ambiguity, timestamp);
                        socket.send(packet);
                         
                    } catch (IOException e) {
                        log("Failed to send packet: " + e);
                    }
                }
                
                try {
                    Thread.sleep(15);
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
            if (bestPose != null) {
                bestPoseLogEntry.append(bestPose.pose().estimatedPose.toPose2d(), getMicroSeconds());
                return bestPose;
            }
            return new VisionInfo(null, null);
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

            //Log all tags
            var tags = latestResult.getTargets();
            long time = getMicroSeconds();
            cameraInfo.logEntry().tagNumEntries.append(tags.stream().mapToLong((tag) -> tag.getFiducialId()).toArray(), time);
            cameraInfo.logEntry().tagPoseEntries.append(tags.stream().map((tag) -> new Pose3d().plus(tag.getBestCameraToTarget())).toArray(Pose3d[]::new), time);
            cameraInfo.logEntry().tagDistanceEntries.append(tags.stream().mapToDouble((tag) -> tag.getBestCameraToTarget().getTranslation().getNorm()).toArray(), time);
            cameraInfo.logEntry().tagAmbiguityEntries.append(tags.stream().mapToDouble((tag) -> tag.getPoseAmbiguity()).toArray(), time);

            PhotonPipelineResult useableResult = latestResult;

            // Create a new result to use -- Using the same metadata as the original latest result
            if (!latestResult.getTargets().stream().allMatch((target) -> filteredTargets.contains(target))) {
                useableResult = new PhotonPipelineResult(
                    latestResult.metadata,
                    filteredTargets,
                    Optional.empty()
                );
            }

            // If pose ambiguity is to high well scrap the result
            boolean highPoseAmbiguity = latestResult.getBestTarget().getPoseAmbiguity() > VisionConstants.POSE_AMBIGUITY_TOLERANCE;

            // If the best tag's distance is too far than scrap the result
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
                cameraInfo.logEntry().robotPoseEntry.append(pose.estimatedPose.toPose2d(), getMicroSeconds());
                
                // Add the vision measurment
                return new VisionInfo(useableResult, pose);
            } else {
                // Get the estimated position
                poseOpt = cameraInfo.poseEstimator().estimateLowestAmbiguityPose(useableResult);
                
                if (poseOpt.isPresent()) {
                    // The pose
                    EstimatedRobotPose pose = poseOpt.get();
                    cameraInfo.logEntry().robotPoseEntry.append(pose.estimatedPose.toPose2d(), getMicroSeconds());

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

        private DatagramPacket getBinaryPacket(Pose2d pose, double ambiguity, double timestamp) throws IllegalArgumentException, UnknownHostException {
            // TODO: UPDATE WITH 8 MORE BITS
            ByteBuffer buffer = ByteBuffer.allocate(40);

            // Add our data to the buffer
            buffer.putDouble(pose.getX());
            buffer.putDouble(pose.getY());
            buffer.putDouble(pose.getRotation().getRadians());
            buffer.putDouble(ambiguity);
            buffer.putDouble(timestamp);

            byte[] data = buffer.array();
            return new DatagramPacket(data, data.length, InetAddress.getByName("10.8.62.2"), 12345);
        }

        @Override
        public void close() throws Exception {
            // Close our connections when the program closes
            socket.close();
            photonNT.close();
        }

        private long getMicroSeconds() {
            return (System.nanoTime() - startTime) / 1000;
        }
    }