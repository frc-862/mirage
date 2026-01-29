// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.RobotMap;
import frc.util.hardware.ThunderBird;

public class Climber extends SubsystemBase {
    private ThunderBird climberMotor;
    private final DutyCycleOut dutyCycle = new DutyCycleOut(0.0);
    private final PositionVoltage positionPID;



    /** Creates a new Climber. */
    public Climber() {
        climberMotor = new ThunderBird(RobotMap.CLIMBER_MOTOR_ID, RobotMap.CAN_BUS,
         ClimberConstants.CLIMBER_MOTOR_INVERTED, ClimberConstants.CLIMBER_MOTOR_STATOR_LIMIT,
          ClimberConstants.CLIMBER_MOTOR_BRAKE_MODE);

        positionPID = new PositionVoltage(0);

}

    @Override
    public void periodic() {
    // This method will be called once per scheduler run

}
}
