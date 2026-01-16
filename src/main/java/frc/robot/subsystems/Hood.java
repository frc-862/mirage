// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.hardware.ThunderBird;
import frc.robot.constants.RobotMap;

public class Hood extends SubsystemBase {

    public final DutyCycleOut dutyCycle = new DutyCycleOut(0.0);

    private ThunderBird hoodMotor;
    /** Creates a new Hood. */
    public Hood() {
        hoodMotor = new ThunderBird(RobotMap.HOOD_MOTOR_ID, HoodConstants.CAN_BUS,
            HoodConstants.HOOD_MOTOR_INVERTED, HoodConstants.HOOD_MOTOR_STATOR_LIMIT,
            HoodConstants.HOOD_MOTOR_BRAKE_MODE);
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

    /**
     * Stops the hood motor.
     */
    public void stop() {
        hoodMotor.stopMotor();
    }
}
