// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.RobotMap;
import frc.util.hardware.ThunderBird;

public class Shooter extends SubsystemBase {

    public final DutyCycleOut dutyCycle = new DutyCycleOut(0.0);
    /**Creates new motors */
    ThunderBird topMotor;
    ThunderBird bottomMotor;

    /** Creates a new Shooter. */
    public Shooter() {
        //Sets new motors
        topMotor = new ThunderBird(ShooterConstants.SHOOTER_TOP_MOTOR_ID, RobotMap.CAN_BUS,
            ShooterConstants.SHOOTER_TOP_MOTOR_INVERTED, ShooterConstants.SHOOTER_TOP_MOTOR_STATOR_LIMIT, ShooterConstants.SHOOTER_TOP_MOTOR_BRAKE);
        bottomMotor = new ThunderBird(ShooterConstants.SHOOTER_BOTTOM_MOTOR_ID, RobotMap.CAN_BUS,
            ShooterConstants.SHOOTER_BOTTOM_MOTOR_INVERTED, ShooterConstants.SHOOTER_BOTTOM_MOTOR_STATOR_LIMIT, ShooterConstants.SHOOTER_BOTTOM_MOTOR_BRAKE);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    /**
     * Set top motor power
     * @param power
    */
    public void setTopPower(double power) {
        topMotor.setControl(dutyCycle.withOutput(power));
    }

    /**
     * Set top motor power
     * @param power
     */
    public void setBottomPower(double power) {
        bottomMotor.setControl(dutyCycle.withOutput(power));
    }

    /**
     * Stoping the top Motor
     */
    public void stopTopMotor() {
        topMotor.stopMotor();
    }

    /**
     * Stoping the bottom motor
     */
    public void stopBottomMotor() {
        bottomMotor.stopMotor();
    }
}
