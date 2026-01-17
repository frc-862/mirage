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

    public final DutyCycleOut dutyCycle;
    /**Creates new motors */
    private ThunderBird topMotor;
    private ThunderBird bottomMotor;

    private VelocityVoltage velocityPid;

    // private Velocity topShooterTarget;
    // private Velocity bottomShooterTarget;

    /** Creates a new Shooter Subsystem. */
    public Shooter() {
        //Sets new motors
        topMotor = new ThunderBird(RobotMap.SHOOTER_TOP_MOTOR_ID, RobotMap.CAN_BUS,
            ShooterConstants.SHOOTER_TOP_MOTOR_INVERTED, ShooterConstants.SHOOTER_TOP_MOTOR_STATOR_LIMIT, ShooterConstants.SHOOTER_TOP_MOTOR_BRAKE);
        bottomMotor = new ThunderBird(RobotMap.SHOOTER_BOTTOM_MOTOR_ID, RobotMap.CAN_BUS,
            ShooterConstants.SHOOTER_BOTTOM_MOTOR_INVERTED, ShooterConstants.SHOOTER_BOTTOM_MOTOR_STATOR_LIMIT, ShooterConstants.SHOOTER_BOTTOM_MOTOR_BRAKE);

        dutyCycle = new DutyCycleOut(0.0);
        velocityPid = new VelocityVoltage(0d);

        TalonFXConfiguration topMotorConfig = topMotor.getConfig();
        topMotorConfig.Slot0.kP = ShooterConstants.kP;
        topMotorConfig.Slot0.kI = ShooterConstants.kI;
        topMotorConfig.Slot0.kD = ShooterConstants.kD;
        topMotorConfig.Slot0.kV = ShooterConstants.kV;
        topMotorConfig.Slot0.kS = ShooterConstants.kS;

        TalonFXConfiguration bottomMotorConfig = bottomMotor.getConfig();
        bottomMotorConfig.Slot0.kP = ShooterConstants.kP;
        bottomMotorConfig.Slot0.kI = ShooterConstants.kI;
        bottomMotorConfig.Slot0.kD = ShooterConstants.kD;
        bottomMotorConfig.Slot0.kV = ShooterConstants.kV;
        bottomMotorConfig.Slot0.kS = ShooterConstants.kS;
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

    /**
     * set top motor velocity
     * @param velocity
     */
    public void setTopVelocity(AngularVelocity velocity){
        topMotor.setControl(velocityPid.withVelocity(velocity));
    }

    /**
     *set bottom motor velocity
     *@param velocity
     */
     public void setBottomVelocity(AngularVelocity velocity){
        bottomMotor.setControl(velocityPid.withVelocity(velocity));
     }

     /**
      * sets Velocity for both motors
      * @param top
      * @param bottom
      */
     public void setBothVelocity(AngularVelocity top, AngularVelocity bottom){
        setTopVelocity(top);
        setBottomVelocity(bottom);
     }
}
