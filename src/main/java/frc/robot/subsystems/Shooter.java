// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.RobotMap;
import frc.util.hardware.ThunderBird;

public class Shooter extends SubsystemBase {

    //creates a shooter motor//
    private ThunderBird shooterMotor;

    private final DutyCycleOut dutyCycle;
    private VelocityVoltage velocityPID;

    private AngularVelocity targetVelocity;

    /** Creates a new Shooter Subsystem. */
    public Shooter() {
        this(new ThunderBird(RobotMap.SHOOTER_MOTOR_ID, RobotMap.CAN_BUS,
            ShooterConstants.SHOOTER_MOTOR_INVERTED, ShooterConstants.SHOOTER_MOTOR_STATOR_LIMIT, ShooterConstants.SHOOTER_MOTOR_BRAKE));
    }

    /**
     * Creates a new Shooter Subsystem with a provided motor.
     * @param motor the ThunderBird motor to use for the shooter
     */
    public Shooter(ThunderBird motor) {
        //Sets the shooter motor
        shooterMotor = motor;

        //instatiates duty cycle and velocity pid
        dutyCycle = new DutyCycleOut(0.0);
        velocityPID = new VelocityVoltage(0d);

        //creates a config for the shooter motor
        TalonFXConfiguration shooterMotorConfig = shooterMotor.getConfig();
        shooterMotorConfig.Slot0.kP = ShooterConstants.kP;
        shooterMotorConfig.Slot0.kI = ShooterConstants.kI;
        shooterMotorConfig.Slot0.kD = ShooterConstants.kD;
        shooterMotorConfig.Slot0.kV = ShooterConstants.kV;
        shooterMotorConfig.Slot0.kS = ShooterConstants.kS;
        shooterMotor.applyConfig(shooterMotorConfig);
    }

    @Override
    public void periodic() {}

    /**
     * Sets motor power of the shooter
     * @param power
     */
    public void setPower(double power) {
        shooterMotor.setControl(dutyCycle.withOutput(power));
    }

    /**
     * Stops all movement of the shooter motor
     */
    public void stopMotor() {
        shooterMotor.stopMotor();
    }

    /**
     * set motor velocity
     * @param velocity
     */
    public void setVelocity(AngularVelocity velocity){
        targetVelocity = velocity;
        shooterMotor.setControl(velocityPID.withVelocity(velocity));
    }

    /**
     * @return the velocity of the shooter motor
     */
    public AngularVelocity getVelocity(){
        return shooterMotor.getVelocity().getValue();
    }

    /**
     * @return whether or not the current velocity is near the target velocity
     */
    public boolean velocityOnTarget(){
        return getVelocity().isNear(targetVelocity, ShooterConstants.TOLERANCE);
    }

}
