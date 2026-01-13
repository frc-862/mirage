// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
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
    CameraThread frontRightThread;
    CameraThread frontLeftThread;
    CameraThread backRightThread;
    CameraThread backLeftThread;

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
        frontRightThread = new CameraThread(CameraInfo.FRONT_RIGHT, frontRightData);
        frontLeftThread = new CameraThread(CameraInfo.FRONT_LEFT, frontLeftData);
        backRightThread = new CameraThread(CameraInfo.BACK_RIGHT, backRightData);
        backLeftThread = new CameraThread(CameraInfo.BACK_LEFT, backLeftData);

        // Start the trheads
        frontRightThread.start();
        frontLeftThread.start();
        backRightThread.start();
        backLeftThread.start();
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
    private class CameraThread extends Thread {

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
        CameraThread(CameraInfo cameraInfo, AtomicReference<VisionUpdate> updateData) {
            this.camera = new PhotonCamera(cameraInfo.name);
            this.updateData = updateData;

            poseEstimator = new PhotonPoseEstimator(VisionConstants.TAG_LAYOUT, cameraInfo.offset);
        }

        @Override
        public void run() {
            // Pause before starting the thread
            try {
                Thread.sleep(1500);
            } catch (InterruptedException e) {
                DataLogManager.log("[BetterPhotonVision] Failed to init sleep");
            }

            //loop
            while (true) {
                try {
                    // Sleep for 15 miliseconds before next loop considering the cameara is updating at 50fps
                    try {
                        Thread.sleep(15);
                    } catch (InterruptedException e) {
                        DataLogManager.log("[PHOTON VISOIN] Failed to sleep");
                    }

                    // All results
                    List<PhotonPipelineResult> results = camera.getAllUnreadResults();

                    // If theres no results just skip this iteration
                    if (results.isEmpty()) {
                        continue;
                    }

                    // Get the latest result of all thme
                    PhotonPipelineResult latestResult = getLatestResult(results);

                    // The result we will actually use
                    PhotonPipelineResult useableResult = latestResult;

                    // Filter out the targets
                    List<PhotonTrackedTarget> filteredTargets = new ArrayList<>(latestResult.getTargets());
                    filteredTargets.removeIf((tag) -> VisionConstants.TAG_IGNORE_LIST.contains((short) tag.getFiducialId()));

                    // Scrap it if the new result has no target
                    if (filteredTargets.isEmpty()) {
                        continue;
                    }

                    // Create a new result to use -- Using the same metadata as the original latest result
                    useableResult = new PhotonPipelineResult(
                        latestResult.metadata,
                        filteredTargets,
                        Optional.empty()
                    );

                    // If the ambiguity is too high skip the iteration
                    if (useableResult.getBestTarget().getPoseAmbiguity() > VisionConstants.POSE_AMBIGUITY_TOLERANCE) {
                        continue;
                    }

                    // If the best tag's distance is too far than scrap the result
                    double bestDistance = useableResult.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();
                    if (bestDistance > VisionConstants.TAG_DISTANCE_TOLERANCE) {
                        continue;
                    }

                    // Get the estimated position
                    Optional<EstimatedRobotPose> poseOpt = poseEstimator.estimateCoprocMultiTagPose(useableResult);
                    if (poseOpt.isEmpty()) {
                        poseOpt = poseEstimator.estimateLowestAmbiguityPose(useableResult); // fallback strategy
                    }

                    // If the estimated position is there run this code
                    if (poseOpt.isPresent()) {
                        // The pose
                        EstimatedRobotPose pose = poseOpt.get();

                        // Add the vision measurment
                        updateData.set(new VisionUpdate(pose, bestDistance));
                    } else {
                        DataLogManager.log("[PHOTON VISION] Pose not available");
                    }
                } catch (Exception e) {
                    DataLogManager.log("[PHOTON VISION] " + e);
                }
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
    }
}
