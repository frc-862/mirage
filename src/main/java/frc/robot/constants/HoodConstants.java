// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import java.util.Map;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MomentOfInertia;

public class HoodConstants {
    public static final boolean INVERTED = false; // temp
    public static final double STATOR_LIMIT = 40d; // temp
    public static final boolean BRAKE = true; // temp

    public static final Angle MIN_ANGLE = Degree.of(0); // Hood v2
    public static final Angle MAX_ANGLE = Degree.of(30); // Hood v2

    public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.1); // Temp

    // Input is distance to target in meters, output is hood angle in degrees
    public static final InterpolatingDoubleTreeMap HOOD_MAP = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(2d, 10d),
        Map.entry(4d, 20d),
        Map.entry(6d, 30d));

    public static final double kS = 0.25d; // temp
    public static final double kV = 2; // temp
    public static final double kA = 0.01; // temp
    public static final double kP = 2; // temp
    public static final double kI = 0.0; // temp
    public static final double kD = 0.0; // temp

    public static final Angle POSITION_TOLERANCE = Degree.of(1); // temp

    // Conversion ratios
    public static final double ROTOR_TO_MECHANISM_RATIO = 25d; // Hood v2
    public static final double ROTOR_TO_ENCODER_RATIO = 1d; // Cancoder mounted on motor
    public static final double ENCODER_TO_MECHANISM_RATIO = ROTOR_TO_MECHANISM_RATIO / ROTOR_TO_ENCODER_RATIO;
}
