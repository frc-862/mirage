// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Pound;
import static edu.wpi.first.units.Units.Pounds;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;

/** Add your docs here. */
public class ClimberConstants {
    public static final boolean CLIMBER_MOTOR_INVERTED = false; // Temp
    public static final double CLIMBER_MOTOR_STATOR_LIMIT = 40d; // Temp
    public static final boolean CLIMBER_MOTOR_BRAKE_MODE = false; // Temp

    public static final double CLIMB_KP = 0.0; // Temp
    public static final double CLIMB_KI = 0.0; // Temp
    public static final double CLIMB_KD = 0.0; // Temp
    public static final double CLIMB_KS = 0.0; // Temp
    public static final double CLIMB_KV = 0.0; // Temp
    public static final double CLIMB_KA = 0.0; // Temp
    public static final double CLIMB_KG = 0.0; // Temp

    public static final double CLIMB_TOLERANCE = 0.5;

    public static final double GEARING_RATIO = 1d; //Temp
    public static final Mass WEIGHT = Pound.of(20); //Temp
    public static final Distance LENGTH = Meter.of(0.2); //Temp
    public static final Distance RADIUS = Meter.of(0.18); //Temp
    public static final Distance MIN_HEIGHT = Meter.of(0.2); //Temp
    public static final Distance MAX_HEIGHT = Meter.of(1); //Temp
    public static final Distance START_HEIGHT  = Meter.of(0.3); //Temp
    public static final double  STANDARD_DEVIATION = 0d;
}
