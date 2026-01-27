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
import static frc.util.Units.clamp;


public class Hood extends SubsystemBase {
    private ThunderBird hoodMotor;

    public final DutyCycleOut dutyCycle;

    // create a Motion Magic request, voltage output
    final MotionMagicVoltage request;
    private Angle targetAngle;

    /** Creates a new Hood Subsystem. */
    public Hood() {
        hoodMotor = new ThunderBird(RobotMap.HOOD, RobotMap.CAN_BUS,
            HoodConstants.INVERTED, HoodConstants.STATOR_LIMIT,
            HoodConstants.BRAKE);

        dutyCycle = new DutyCycleOut(0d);

        request = new MotionMagicVoltage(0d);

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
    public void periodic() {}

    /**
     * Set the power of the hood motor using duty cycle out
     * @param power duty cycle value from -1.0 to 1.0
     */
    public void setPower(double power) {
        hoodMotor.setControl(dutyCycle.withOutput(power));
    }

    /**
     * Sets position of the hood
     * @param position in degrees
     */
    public void setPosition(Angle position) {
        targetAngle = clamp(position, HoodConstants.MIN_ANGLE, HoodConstants.MAX_ANGLE);

        hoodMotor.setControl(request.withPosition(targetAngle));
    }

    /**
     * Gets the current angle of the hood
     * @return
     * current angle
     */
    public Angle getAngle() {
        return hoodMotor.getPosition().getValue();
    }

    /**
     * Gets the target angle of the hood
     * @return
     * targetAngle
     */
    public Angle getTargetAngle() {
        return targetAngle;
    }

    /**
     * Returns true if the hood is on target
     * @return
     * True if on target, false otherwise
     */
    public boolean isOnTarget() {
        return getAngle().isNear(getTargetAngle(), HoodConstants.POSITION_TOLERANCE);
    }

    /**
     * Stops all movement to the hood motor
     */
    public void stop() {
        hoodMotor.stopMotor();
    }
}
