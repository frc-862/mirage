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

    //creates a Duty cycle out variable//
    private final DutyCycleOut dutyCycle;

    //creates a velocity PID variable//
    private VelocityVoltage velocityPID;

    private AngularVelocity targetVelocity;

    /** Creates a new Shooter Subsystem. */
    public Shooter() {
        //Sets new motors
        shooterMotor = new ThunderBird(RobotMap.SHOOTER_TOP_MOTOR_ID, RobotMap.CAN_BUS,
            ShooterConstants.SHOOTER_MOTOR_INVERTED, ShooterConstants.SHOOTER_MOTOR_STATOR_LIMIT, ShooterConstants.SHOOTER_MOTOR_BRAKE);

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
    }

    @Override
    public void periodic() {}

    /**
     * Sets bottom motor power of the shooter
     * @param power
     */
    public void setPower(double power) {
        shooterMotor.setControl(dutyCycle.withOutput(power));
    }

    /**
     * Stops all movement of the top shooter motor
     */
    public void stopMotor() {
        shooterMotor.stopMotor();
    }

    /**
     * set top motor velocity
     * @param velocity
     */
    public void setVelocity(AngularVelocity velocity){
        targetVelocity = velocity;
        shooterMotor.setControl(velocityPID.withVelocity(velocity));
    }

    //gets the velocity of the shooter motor as an Angular Velocity
    public AngularVelocity getVelocity(){
        return shooterMotor.getVelocity().getValue();
    }

    /**uses the target velocity set earlier and checks to make sure that the velocity
    is near where we want it to be with the Tolerance given*/
    public boolean velocityOnTarget(){
        return getVelocity().isNear(targetVelocity, ShooterConstants.TOLERANCE);
    }

}
