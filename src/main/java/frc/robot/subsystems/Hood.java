// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.hardware.ThunderBird;
import frc.robot.constants.RobotMap;
import frc.robot.constants.HoodConstants;
import edu.wpi.first.units.measure.Angle;

public class Hood extends SubsystemBase {

    public final DutyCycleOut dutyCycle = new DutyCycleOut(0.0);

    private ThunderBird hoodMotor;

    // create a Motion Magic request, voltage output
    final MotionMagicVoltage request = new MotionMagicVoltage(0);
    /** Creates a new Hood. */
    public Hood() {
        hoodMotor = new ThunderBird(RobotMap.HOOD_MOTOR_ID, RobotMap.CAN_BUS,
            HoodConstants.HOOD_MOTOR_INVERTED, HoodConstants.HOOD_MOTOR_STATOR_LIMIT,
            HoodConstants.HOOD_MOTOR_BRAKE_MODE);

        // in init function
        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = HoodConstants.kS; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = HoodConstants.kV; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = HoodConstants.kA; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = HoodConstants.kP; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = HoodConstants.kI; // no output for integrated error
        slot0Configs.kD = HoodConstants.kD; // A velocity error of 1 rps results in 0.1 V output
        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = HoodConstants.CRUISE_VELOCITY; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = HoodConstants.ACCELERATION; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = HoodConstants.JERK; // Target jerk of 1600 rps/s/s (0.1 seconds)

        hoodMotor.applyConfig(talonFXConfigs);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    /**
     * Sets power to (-1 to 1) to the hood motor.
     * @param power
     */
    public void setPower(double power) {
        hoodMotor.setControl(dutyCycle.withOutput(power));
    }

    public void setPosition(Angle position) {
        hoodMotor.setControl(request.withPosition(position));
    }

    /**
     * Stops the hood motor.
     */
    public void stop() {
        hoodMotor.stopMotor();
    }
}
