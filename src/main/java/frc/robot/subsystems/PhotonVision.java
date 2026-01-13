// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;

public class PhotonVision extends SubsystemBase {
    // The drivetrain to add vision measurments
    Swerve drivetrain;

    // mac
    MacMini mac;

    // Atomic
    AtomicReference<EstimatedRobotPose> pose;

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

        executor1.submit(() -> {try { pose.set(executor2.submit(() -> mac.getVisionPose()).get()); } catch (Exception e) {}});
    }
 
    @Override
    public void periodic() {
        if (pose.get() != null) {
            drivetrain.addVisionMeasurement(pose.get().estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(pose.get().timestampSeconds));
        }
    }

    // -------------------------------------

    private class MacMini {
        // Camera info
        private record CameraInfo(PhotonCamera camera, PhotonPoseEstimator poseEstimator) {};
        private record VisionInfo(PhotonPipelineResult result, EstimatedRobotPose pose) {};

        // Cameras
        CameraInfo[] cameras;

        public MacMini() {            
            // cameras
            cameras = new CameraInfo[VisionConstants.CAMERA_CONSTANTS.length];
            
            // Create the camears
            for (int i = 0; i < VisionConstants.CAMERA_CONSTANTS.length; i++) {
                PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField), PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_RIO, VisionConstants.CAMERA_CONSTANTS[i].offset());
                poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);

                cameras[i] = new CameraInfo(new PhotonCamera(VisionConstants.CAMERA_CONSTANTS[i].name()), poseEstimator);
            }

        }

        public EstimatedRobotPose getVisionPose() {
            try {
                VisionInfo[] poses = new VisionInfo[VisionConstants.CAMERA_CONSTANTS.length];
                
                for (int i = 0; i < VisionConstants.CAMERA_CONSTANTS.length; i++) {
                   poses[i] = getVisionPose(cameras[i]);
                }

                return getBestPose(poses).pose;
            } catch (Exception e) {
                System.out.println("Boo hoo");
                return null;
            }
        }

        // Gets the latest result from multiple results
        private PhotonPipelineResult getLatestResult(List<PhotonPipelineResult> results) {
            int latestResultIndex = 0;

            for (int i = 0; i < results.size(); i++) {
                if (results.get(i).metadata.getCaptureTimestampMicros() > results.get(latestResultIndex).metadata.getCaptureTimestampMicros()) {
                    latestResultIndex = i;
                }
            }

            PhotonPipelineResult latestResult = results.get(latestResultIndex);

            return latestResult;
        }

        private VisionInfo getBestPose(VisionInfo[] visionInfos) {
            VisionInfo bestPose = null;

            for (VisionInfo info : visionInfos) {
                if (bestPose == null) {
                    bestPose = info;
                }

                if (bestPose.result().getBestTarget().poseAmbiguity > info.result().getBestTarget().poseAmbiguity) {
                    bestPose = info;
                }
            }

            return bestPose;
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

            // Create a new result to use -- Using the same metadata as the original latest result
            PhotonPipelineResult useableResult = new PhotonPipelineResult(
                latestResult.metadata,
                filteredTargets,
                Optional.empty()
            );

            // If pose ambiguity is to high well scrap the result
            boolean highPoseAmbiguity = useableResult.getBestTarget().getPoseAmbiguity() > VisionConstants.POSE_AMBIGUITY_TOLERANCE;

            // If the best tag's distance is too far than scrap the result
            double bestDistance = useableResult.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();
            boolean highDistance = bestDistance > VisionConstants.TAG_DISTANCE_TOLERANCE;

            if (highPoseAmbiguity || highDistance) {
                return null;
            }
            
            // Get the estimated position
            Optional<EstimatedRobotPose> poseOpt = cameraInfo.poseEstimator().update(useableResult);

            // If the estimated position is there run this code
            if (poseOpt.isPresent()) {
                // The pose
                EstimatedRobotPose pose = poseOpt.get();
                
                // Add the vision measurment
                return new VisionInfo(useableResult, pose);
            } else {
                System.out.println("[PHOTON VISION] Pose not available");
            }

            return null;
        }
    }
}