// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SpindexerConstants;
import frc.robot.constants.RobotMap;
import frc.util.hardware.ThunderBird;

public class Transfer extends SubsystemBase {
  ThunderBird spindexerTower;

  /** Creates a new Transfer. */
  public Transfer() {
    spindexerTower = new ThunderBird(RobotMap.TRANSFER,
    RobotMap.CAN_BUS, SpindexerConstants.SPINDEXER_MOTOR_INVERTED,
    SpindexerConstants.SPINDEXER_MOTOR_STATOR_LIMIT,
    SpindexerConstants.SPINDEXER_MOTOR_BRAKE_MODE);
  }

  /* Turns the motor for the tower on */
  public void setPower(double power) {
    spindexerTower.setControl(new DutyCycleOut(power));
    }

  /* Stops the spindexer tower  */
  public void stop() {
    spindexerTower.stopMotor();
    }
}