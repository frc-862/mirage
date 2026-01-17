// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.RobotMap;
import frc.util.hardware.ThunderBird;

public class Indexer extends SubsystemBase {
    ThunderBird indexerMotor;

    /** Creates a new Indexer. */
    public Indexer() {
        indexerMotor = new ThunderBird(RobotMap.INDEXER_MOTOR_ID, RobotMap.CAN_BUS,
            IndexerConstants.INDEXER_MOTOR_INVERTED, IndexerConstants.INDEXER_MOTOR_STATOR_LIMIT, IndexerConstants.INDEXER_MOTOR_BRAKE_MODE);
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    /**
     * Sets power to (-1 to 1) to the indexer motor.
     * @param power
     */
    public void setPower(double power) {
        indexerMotor.setControl(new DutyCycleOut(power));
    }

    /**
     * Stops the indexer motor.
     */
    public void stop() {
        indexerMotor.stopMotor();
    }
}
