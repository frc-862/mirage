// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.TransferConstants;
import frc.robot.constants.RobotMap;
import frc.util.hardware.ThunderBird;

public class Transfer extends SubsystemBase {
    ThunderBird transferMotor;

    DutyCycleOut dutyCycle;

    /** Creates a new Transfer Subsystem. */
    public Transfer() {
        transferMotor = new ThunderBird(RobotMap.TRANSFER_MOTOR_ID,
                RobotMap.CAN_BUS, TransferConstants.INVERTED,
                TransferConstants.STATOR_LIMIT,
                TransferConstants.BRAKE_MODE);

        dutyCycle = new DutyCycleOut(0.0);
    }

    /* Turns the motor for the tower on */
    public void setPower(double power) {
        transferMotor.setControl(dutyCycle.withOutput(power));
    }

    /* Stops the spindexer tower */
    public void stop() {
        transferMotor.stopMotor();
    }
}