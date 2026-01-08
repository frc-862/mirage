// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants.CameraPosition;

public class PhotonVision extends SubsystemBase {
    /** Creates a new PhotonVision. */
    public PhotonVision() {

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    /**
     * Camera thread to run cameras in parrellel
     */

    private class CameraThread extends Thread {

        PhotonCamera camera;

        public CameraThread(CameraPosition cameraPosition) {

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

                // Sleep for 15 miliseconds before next loop considering the cameara is updating at 50fps
                try {
                    Thread.sleep(15);
                } catch (InterruptedException e) {
                    DataLogManager.log("[BetterPhotonVision] Failed to sleep");
                }
            }
        }
    }
}
