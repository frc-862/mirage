// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.units.measure.AngularVelocity;
import static edu.wpi.first.units.Units.RotationsPerSecond;

/** Add your docs here. */
public class ShooterConstants {

    public static final boolean SHOOTER_MOTOR_INVERTED = false; //temp
    public static final double SHOOTER_MOTOR_STATOR_LIMIT = 120.0; //temp
    public static final boolean SHOOTER_MOTOR_BRAKE = false; //temp

    public static final double kP = 0d;
    public static final double kI = 0d;
    public static final double kD = 0d;
    public static final double kV = 0d;
    public static final double kS = 0d;
    public static final AngularVelocity TOLERANCE = RotationsPerSecond.of(2);
}
