// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotMap;
import frc.robot.constants.TurretConstants;
import frc.util.hardware.ThunderBird;

public class Transfer extends SubsystemBase {
  ThunderBird transfer;

  /** Creates a new Transfer. */
  public Transfer() {
    transfer = new ThunderBird(RobotMap.TRANSFER_MOTOR_ID,
    RobotMap.CAN_BUS, TurretConstants.TRANSFER_MOTOR_INVERTED,
    TurretConstants.TRANSFER_MOTOR_STATOR_LIMIT,
    TurretConstants.TRANSFER_MOTOR_BRAKE_MODE);
  }

  /* Turns the motor for the transfer on */
  public void setPower(double power) {
    transfer.setControl(new DutyCycleOut(power));
    }

  /* Stops the transfer motor  */
  public void stop() {
    transfer.stopMotor();
    }
}