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

    private final DutyCycleOut dutyCycle = new DutyCycleOut(0.0);

    private ThunderBird topMotor;
    private ThunderBird bottomMotor;

    /** Creates a new Shooter Subsystem. */
    public Shooter() {
        topMotor = new ThunderBird(ShooterConstants.SHOOTER_TOP_MOTOR_ID, RobotMap.CAN_BUS,
            ShooterConstants.SHOOTER_TOP_MOTOR_INVERTED, ShooterConstants.SHOOTER_TOP_MOTOR_STATOR_LIMIT, ShooterConstants.SHOOTER_TOP_MOTOR_BRAKE);
        bottomMotor = new ThunderBird(ShooterConstants.SHOOTER_BOTTOM_MOTOR_ID, RobotMap.CAN_BUS,
            ShooterConstants.SHOOTER_BOTTOM_MOTOR_INVERTED, ShooterConstants.SHOOTER_BOTTOM_MOTOR_STATOR_LIMIT, ShooterConstants.SHOOTER_BOTTOM_MOTOR_BRAKE);
    }

    @Override
    public void periodic() {}

    /**
     * Sets top motor power of the shooter
     * @param power
    */
    public void setTopPower(double power) {
        topMotor.setControl(dutyCycle.withOutput(power));
    }

    /**
     * Sets bottom motor power of the shooter
     * @param power
     */
    public void setBottomPower(double power) {
        bottomMotor.setControl(dutyCycle.withOutput(power));
    }

    /**
     * Stops all movement of the top shooter motor
     */
    public void stopTopMotor() {
        topMotor.stopMotor();
    }

    /**
     * Stops all movement of the bottom shooter motor
     */
    public void stopBottomMotor() {
        bottomMotor.stopMotor();
    }
}
