// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.RobotMap;
import frc.util.hardware.ThunderBird;

public class TurretTower extends SubsystemBase {
  ThunderBird indexerTower;

  /** Creates a new TurretTower. */
  public TurretTower() {
    indexerTower = new ThunderBird(RobotMap.INDEXER_MOTOR_ID, 
    RobotMap.CAN_BUS, IndexerConstants.INDEXER_MOTOR_INVERTED, 
    IndexerConstants.INDEXER_MOTOR_STATOR_LIMIT, 
    IndexerConstants.INDEXER_MOTOR_BRAKE_MODE);
  }
 
  /* Turns the motor for the tower on */
  public void setPower(double power) {
    indexerTower.setControl(new DutyCycleOut(power));
    }

  /* Stops the indexer tower  */
  public void stop() {
    indexerTower.stopMotor();
    }
}