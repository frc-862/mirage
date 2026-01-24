// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

/** Constants for Hood */
public class HoodConstants {
    public static final boolean INVERTED = false; // Temp
    public static final double STATOR_LIMIT = 40d; // Temp
    public static final boolean BRAKE = false; // Temp
    public static final double HOOD_MOTOR_POWER = 1d;

    // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/motion-magic.html
    public static final double kS = 0.0; // TEMP
    public static final double kV = 0.0; // TEMP
    public static final double kA = 0.0; // TEMP
    public static final double kP = 0.0; // TEMP
    public static final double kI = 0.0; // TEMP
    public static final double kD = 0.0; // TEMP
    // Motion Magic parameters (https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/motion-magic.html)
    public static final double CRUISE_VELOCITY = 80.0; // TEMP
    public static final double ACCELERATION = 160.0; // TEMP
    public static final double JERK = 1600.0; // TEMP
    // Angle limits
    public static final Angle MIN_ANGLE = Degrees.of(0); // temp
    public static final Angle MAX_ANGLE = Degrees.of(90); // temp
    // Tolerance for being "on target"
    public static final Angle POSITION_TOLERANCE = Degrees.of(0.41); // temp

}
