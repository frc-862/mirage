// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterConstants {
    public static final boolean INVERTED = false; // temp
    public static final double STATOR_LIMIT = 120.0; // temp
    public static final boolean BRAKE = false; // temp

    public static final double kP = 0.1d;
    public static final double kI = 0d;
    public static final double kD = 0d;
    public static final double kV = 0.12d;
    public static final double kS = 0.5d;
    public static final AngularVelocity TOLERANCE = RotationsPerSecond.of(2);

    public static final double GEAR_RATIO = 1d; // temp
    public static final Distance FLYWHEEL_CIRCUMFERENCE = Inches.of(4).times(Math.PI).times(2);

    // Input is distance to target in meters, output is shooter speed in rotations per second
    public static final InterpolatingDoubleTreeMap VELOCITY_MAP = InterpolatingDoubleTreeMap.ofEntries(
            Map.entry(2d, 20d),
            Map.entry(4d, 40d),
            Map.entry(6d, 60d));

    // Sim
    public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.05); // temp
    public static final Translation2d SHOOTER_POSITION_ON_ROBOT = new Translation2d(Inches.of(0), Inches.of(9));
    public static final Distance SHOOTER_HEIGHT = Inches.of(18);
    public static final Time MAX_SHOOTING_PERIOD = Seconds.of(0.1); // 10 balls per second
}
