// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

/** Add your docs here. */
public class ShooterConstants {
    // Shooter constants
    public static final boolean SHOOTER_BOTTOM_MOTOR_INVERTED = false; //temp
    public static final boolean SHOOTER_TOP_MOTOR_INVERTED = false; //temp
    public static final double SHOOTER_BOTTOM_MOTOR_STATOR_LIMIT = 120.0; //temp
    public static final double SHOOTER_TOP_MOTOR_STATOR_LIMIT = 120.0; //temp
    public static final boolean SHOOTER_BOTTOM_MOTOR_BRAKE = true; //temp
    public static final boolean SHOOTER_TOP_MOTOR_BRAKE = true; //temp

    public static final double kP = 0d;
    public static final double kI = 0d;
    public static final double kD = 0d;
    public static final double kV = 0d;
    public static final double kS = 0d;
    // public static final AngularVelocity TOLERANCE = RotationsPerSecond.of(2);
    // public static final Current THRESHHOLD = Amps.of(20);
}
