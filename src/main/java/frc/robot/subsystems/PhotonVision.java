// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.shuffleboard.LightningShuffleboard;

public class PhotonVision extends SubsystemBase {

    public class VisionConstants {

        public static final List<Short> TAG_IGNORE_LIST = List.of();

        // TODO: Update this to be the correect field layout for this season,
        public static final AprilTagFieldLayout DEFAULT_TAG_LAYOUT = AprilTagFieldLayout
                .loadField(AprilTagFields.k2026RebuiltWelded);

        public static final double POSE_AMBIGUITY_TOLERANCE = 1;
        public static final double TAG_DISTANCE_TOLERANCE = 10;

        public record CameraConstant(String name, Transform3d offset) {};

        public static final CameraConstant[] CAMERA_CONSTANTS = new CameraConstant[] {
            new CameraConstant("leftCam",
                new Transform3d(
                    Inches.of(11.25),   // forward
                    Inches.of(11.25),   // LEFT
                    Inches.of(10.5),    // up
                    new Rotation3d(
                        0.0,
                        Math.toRadians(-15),  // pitch up
                        Math.toRadians(45)    // yaw outward (left)
                    )
                )
            ),
        };


    }

    private record VisionInfo(PhotonPipelineResult result, EstimatedRobotPose pose) {};

    // The drivetrain to add vision measurments
    Swerve drivetrain;

    // mac
    MacMini mac;

    // Atomic
    AtomicReference<VisionInfo> pose;

    // executor
    ScheduledExecutorService executor1;
    ExecutorService executor2;


    /** Creates a new PhotonVision.
     * 
     * @param drivetrain The main drivetrain on the robot
     */
    public PhotonVision(Swerve drivetrain) {
        this.drivetrain = drivetrain;
        mac = new MacMini();

        executor1 = Executors.newSingleThreadScheduledExecutor();
        executor2 = Executors.newSingleThreadExecutor();

        pose = new AtomicReference<>(null);

        LightningShuffleboard.setPose2d("vision", "vision_pose", new Pose2d());

        executor1.schedule(() -> {
            while (true) { 
                try {
                    VisionInfo result = executor2.submit(() -> mac.getEstimatedPose()).get();
                    if (result != null) {
                        pose.set(result);
                    }
                } catch (Exception e) {
                    e.printStackTrace();
                    log("Failed to get data");
                }
                }
        }, 0, TimeUnit.MILLISECONDS);
    }
 
    @Override
    public void periodic() {
        if (pose.get() != null) {
            VisionInfo updatedPose = pose.getAndSet(null);

            double bestTagAmbiguity = updatedPose.result.getBestTarget().poseAmbiguity;
             LightningShuffleboard.setPose2d("vision", "vision_pose", updatedPose.pose.estimatedPose.toPose2d());
            drivetrain.addVisionMeasurement(
                updatedPose.pose.estimatedPose.toPose2d(), 
                Utils.fpgaToCurrentTime(updatedPose.pose.timestampSeconds), 
                VecBuilder.fill(bestTagAmbiguity*1.7, bestTagAmbiguity*1.7, bestTagAmbiguity*1.7));
            log("Added vision measurment");
        }
    }

    // -------------------------------------

    private class MacMini {
        // Camera info
        private record CameraInfo(PhotonCamera camera, PhotonPoseEstimator poseEstimator) {}; 
        // private record VisionInfo(PhotonPipelineResult result, EstimatedRobotPose pose) {};

        // Cameras
        CameraInfo[] cameras;

        MacMini() {            
            this.cameras = new CameraInfo[VisionConstants.CAMERA_CONSTANTS.length];
            
            // Create the cameras
            for (int i = 0; i < VisionConstants.CAMERA_CONSTANTS.length; i++) {
                AprilTagFieldLayout fieldLayout;

                try {
                    // Get the path to the field from the deploy directory
                    Path fieldPath = Filesystem.getDeployDirectory()
                        .toPath()
                        .resolve("field0120.json");

                    fieldLayout = new AprilTagFieldLayout(fieldPath);
                } catch (Exception e) {
                    // Just use the default field if we can't get it
                    DataLogManager.log("[PHOTON VISION] Can't load field resource-- using default field");
                    fieldLayout = VisionConstants.DEFAULT_TAG_LAYOUT;
                }

                // Get the pose estimator
                PhotonPoseEstimator poseEstimator =
                        new PhotonPoseEstimator(
                                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded),
                                VisionConstants.CAMERA_CONSTANTS[i].offset()
                        );
                    
                // Create the camera using the name from our constant
                PhotonCamera camera = new PhotonCamera(VisionConstants.CAMERA_CONSTANTS[i].name());

                // Create the camera
                this.cameras[i] = new CameraInfo(camera, poseEstimator);
                log(VisionConstants.CAMERA_CONSTANTS[i].name() + " info created sucessfully");
            }

        }

        public VisionInfo getEstimatedPose() {
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
                // log("Failed to get pose");
                e.printStackTrace();
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
            // log("Got best pose");
            return bestPose;
        }

        private VisionInfo getVisionPose(CameraInfo cameraInfo) {
            PhotonCamera camera = cameraInfo.camera;

            List<PhotonPipelineResult> results = camera.getAllUnreadResults();

            // If theres no results just skip this iteration
            if (results.isEmpty()) {
                // log(cameraInfo.camera.getName() + "'s Result is null");
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
                
                // log("Used multitag result");

                // Add the vision measurment
                return new VisionInfo(useableResult, pose);
            } else {
                // Get the estimated position
                poseOpt = cameraInfo.poseEstimator().estimateLowestAmbiguityPose(useableResult);
                
                if (poseOpt.isPresent()) {
                    // The pose
                    EstimatedRobotPose pose = poseOpt.get();

                    // log("Used singletag result");

                    // Add the vision measurment
                    return new VisionInfo(useableResult, pose);
                }
            }

            // We have no pose if were here
            log("No pose");
            return null;
        }
    }

    // im lazy
    private void log(String message) {
        DataLogManager.log("[PHOTON VISION]" + message);
    }
}