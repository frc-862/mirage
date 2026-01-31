// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.RobotMap;
import frc.util.hardware.ThunderBird;

public class Climber extends SubsystemBase {
    private ThunderBird climberMotor;
    private final DutyCycleOut dutyCycle;
    private final PositionVoltage positionPID;

    /** Creates a new Climber Subsystem. */
    public Climber() {
        climberMotor = new ThunderBird(RobotMap.CLIMBER, RobotMap.CAN_BUS,
            ClimberConstants.CLIMBER_MOTOR_INVERTED, ClimberConstants.CLIMBER_MOTOR_STATOR_LIMIT,
            ClimberConstants.CLIMBER_MOTOR_BRAKE_MODE);

        dutyCycle = new DutyCycleOut(0.0);
        positionPID = new PositionVoltage(0);

        TalonFXConfiguration config = climberMotor.getConfig();
        config.Slot0.kP = ClimberConstants.CLIMB_KP;
        config.Slot0.kI = ClimberConstants.CLIMB_KI;
        config.Slot0.kD = ClimberConstants.CLIMB_KD;
        config.Slot0.kS = ClimberConstants.CLIMB_KS;
        config.Slot0.kV = ClimberConstants.CLIMB_KV;
        config.Slot0.kA = ClimberConstants.CLIMB_KA;
        config.Slot0.kG = ClimberConstants.CLIMB_KG;
        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    }

    /**
     * Sets the power output of the climber motor.
     *
     * @param power the power output as a value between -1.0 and 1.0
     */
    public void setPower(double power) {
        dutyCycle.Output = power;
        climberMotor.setControl(dutyCycle);
    }

    /**
     * Sets the target position of the climber motor.
     *
     * @param positionVolts the target position in volts
     */
    public void setPosition(double positionVolts) {
        positionPID.Position = positionVolts;
        climberMotor.setControl(positionPID);
    }

    /**
     * gets the position of the climber
     * @return climber position
     */
    public double getTargetPosition() {
        return positionPID.Position;
    }

    /**
     * gets if the climber is on target
     * @return if climber is on target
     */
    public boolean onTarget() {
        return positionPID.getPositionMeasure().isNear(climberMotor.getPosition().getValue(), ClimberConstants.CLIMB_TOLERANCE);
    }
}
