// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.VisionConstants.CameraInfo;

public class PhotonVision extends SubsystemBase {
    // The drivetrain to add vision measurments
    Swerve drivetrain;

    // Vision update type
    private record VisionUpdate(EstimatedRobotPose pose, double distance) {}
    
    // Vision data
    AtomicReference<VisionUpdate> frontRightData;
    AtomicReference<VisionUpdate> frontLeftData;
    AtomicReference<VisionUpdate> backRightData;
    AtomicReference<VisionUpdate> backLeftData;

    // Vision threads
    CameraExecutor frontRightExecutor;
    CameraExecutor frontLeftExecutor;
    CameraExecutor backRightExecutor;
    CameraExecutor backLeftExecutor;

    /** Creates a new PhotonVision.
     * 
     * @param drivetrain The main drivetrain on the robot
     */
    public PhotonVision(Swerve drivetrain) {
        this.drivetrain = drivetrain;

        // initialize the records
        frontRightData = new AtomicReference<>();
        frontLeftData = new AtomicReference<>();
        backRightData = new AtomicReference<>();
        backLeftData = new AtomicReference<>();

        // threads
        frontRightExecutor = new CameraExecutor(CameraInfo.FRONT_RIGHT, frontRightData);
        frontLeftExecutor = new CameraExecutor(CameraInfo.FRONT_LEFT, frontLeftData);
        backRightExecutor = new CameraExecutor(CameraInfo.BACK_RIGHT, backRightData);
        backLeftExecutor = new CameraExecutor(CameraInfo.BACK_LEFT, backLeftData);

        // Start the trheads
        frontRightExecutor.run();
        frontLeftExecutor.run();
        backRightExecutor.run();
        backLeftExecutor.run();
    }
 
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateVision();
    }

    private void updateVision() {
        // Front right data
        VisionUpdate update = frontRightData.getAndSet(null);

        if (update != null) {
            drivetrain.addVisionMeasurement(update.pose, update.distance);
        }

        // Front left data
        update = frontLeftData.getAndSet(null);
        if (update != null) {
            drivetrain.addVisionMeasurement(update.pose, update.distance);
        }

        // Back right grade
        update = backRightData.getAndSet(null);
        if (update != null) {
             drivetrain.addVisionMeasurement(update.pose, update.distance);
        }

        // Back left grade
        update = backLeftData.getAndSet(null);
        if (update != null) {
             drivetrain.addVisionMeasurement(update.pose, update.distance);
        }
    } 

    // Camera thread to run cameras in parrellel
    private class CameraExecutor {
        // Executor
        ScheduledExecutorService executor;

        // Pose estimator
        PhotonPoseEstimator poseEstimator;

        // Camera stuff
        PhotonCamera camera;
        AtomicReference<VisionUpdate> updateData;

        /**
         * A thread that will run camera operations
         * @param cameraInfo The camera info enum for the specific camera on the robot
         * @param updateData The data that will be used to store camera updates
         */
        CameraExecutor(CameraInfo cameraInfo, AtomicReference<VisionUpdate> updateData) {
            this.camera = new PhotonCamera(cameraInfo.name);
            this.updateData = updateData;

            poseEstimator = new PhotonPoseEstimator(
                VisionConstants.TAG_LAYOUT, 
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                cameraInfo.offset);

            poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
            
            executor = Executors.newSingleThreadScheduledExecutor((r) -> {
                Thread thread = new Thread(r);
                thread.setName(cameraInfo.toString());

                return thread;
            });
        } 

        public void run() {
            //loop
            executor.scheduleWithFixedDelay((() -> {
                try {
                    // All results
                    List<PhotonPipelineResult> results = camera.getAllUnreadResults();

                    // If theres no results just skip this iteration
                    if (!results.isEmpty()) {
                        // Get the latest result of all thme
                        PhotonPipelineResult latestResult = getLatestResult(results);

                        // Filter out the targets
                        List<PhotonTrackedTarget> filteredTargets = new ArrayList<>(latestResult.getTargets());
                        filteredTargets.removeIf((tag) -> VisionConstants.TAG_IGNORE_LIST.contains((short) tag.getFiducialId()));

                        // Scrap it if the new result has no target
                        if (!filteredTargets.isEmpty()) {
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

                            if (!highPoseAmbiguity && !highDistance) {
                                // Get the estimated position
                                Optional<EstimatedRobotPose> poseOpt = poseEstimator.update(useableResult);

                                // If the estimated position is there run this code
                                if (poseOpt.isPresent()) {
                                    // The pose
                                    EstimatedRobotPose pose = poseOpt.get();
                                    
                                    // Add the vision measurment
                                    updateData.set(new VisionUpdate(pose, bestDistance));
                                } else {
                                    DataLogManager.log("[PHOTON VISION] Pose not available");
                                }
                            }
                        }
                    }
                } catch (Exception e) {
                    DataLogManager.log("[PHOTON VISION] " + e);
                }
            }), 1500, 15, TimeUnit.MILLISECONDS);
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
    }
}
