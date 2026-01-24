// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;

/** Constants for Hood */
public class HoodConstants {
    public static final boolean INVERTED = false; // Temp
    public static final double STATOR_LIMIT = 40d; // Temp
    public static final boolean BRAKE = false; // Temp
    public static final double HOOD_MOTOR_POWER = 1d;

    public static final Angle MIN_ANGLE = Degree.of(-90);
    public static final Angle MAX_ANGLE = Degree.of(90);

    public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.086);
    public static final Distance LENGTH = Meter.of(0.18);
    public static final double GEARING_RATIO = 5d; // temp

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
}
