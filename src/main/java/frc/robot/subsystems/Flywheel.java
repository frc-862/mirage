// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FlywheelConstants;
import frc.robot.constants.RobotMap;
import frc.util.hardware.ThunderBird;

public class Flywheel extends SubsystemBase {

    //creates a flywheel motor//
    private ThunderBird flywheelMotor;

    private final DutyCycleOut dutyCycle;
    private VelocityVoltage velocityPID;

    private AngularVelocity targetVelocity;

    /** Creates a new Flywheel Subsystem. */
    public Flywheel() {
        this(new ThunderBird(RobotMap.SHOOTER, RobotMap.CAN_BUS,
            FlywheelConstants.FLYWHEEL_MOTOR_INVERTED, FlywheelConstants.FLYWHEEL_MOTOR_STATOR_LIMIT, FlywheelConstants.FLYWHEEL_MOTOR_BRAKE));
    }

    /**
     * Creates a new Flywheel Subsystem with a provided motor.
     * @param motor the ThunderBird motor to use for the flywheel
     */
    public Flywheel(ThunderBird motor) {
        //Sets the flywheel motor
        flywheelMotor = motor;

        //instatiates duty cycle and velocity pid
        dutyCycle = new DutyCycleOut(0.0);
        velocityPID = new VelocityVoltage(0d);

        //creates a config for the flywheel motor
        TalonFXConfiguration flywheelMotorConfig = flywheelMotor.getConfig();
        flywheelMotorConfig.Slot0.kP = FlywheelConstants.kP;
        flywheelMotorConfig.Slot0.kI = FlywheelConstants.kI;
        flywheelMotorConfig.Slot0.kD = FlywheelConstants.kD;
        flywheelMotorConfig.Slot0.kV = FlywheelConstants.kV;
        flywheelMotorConfig.Slot0.kS = FlywheelConstants.kS;
        flywheelMotor.applyConfig(flywheelMotorConfig);
    }

    @Override
    public void periodic() {}

    /**
     * Sets motor power of the flywheel
     * @param power
     */
    public void setPower(double power) {
        flywheelMotor.setControl(dutyCycle.withOutput(power));
    }

    /**
     * Stops all movement of the flywheel motor
     */
    public void stopMotor() {
        flywheelMotor.stopMotor();
    }

    /**
     * set motor velocity
     * @param velocity
     */
    public void setVelocity(AngularVelocity velocity){
        targetVelocity = velocity;
        flywheelMotor.setControl(velocityPID.withVelocity(velocity));
    }

    /**
     * @return the velocity of the flywheel motor
     */
    public AngularVelocity getVelocity(){
        return flywheelMotor.getVelocity().getValue();
    }

    /**
     * @return whether or not the current velocity is near the target velocity
     */
    public boolean velocityOnTarget(){
        return getVelocity().isNear(targetVelocity, FlywheelConstants.TOLERANCE);
    }

}
