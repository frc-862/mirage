// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pound;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class ClimberConstants {
    public static final boolean CLIMBER_MOTOR_INVERTED = false; // temp
    public static final double CLIMBER_MOTOR_STATOR_LIMIT = 40d; // temp
    public static final boolean CLIMBER_MOTOR_BRAKE_MODE = false; // temp

    public static final double CLIMB_KP = 0.0; // temp
    public static final double CLIMB_KI = 0.0; // temp
    public static final double CLIMB_KD = 0.0; // temp
    public static final double CLIMB_KS = 0.0; // temp
    public static final double CLIMB_KV = 0.0; // temp
    public static final double CLIMB_KA = 0.0; // temp
    public static final double CLIMB_KG = 0.0; // temp

    public static final double CLIMB_TOLERANCE = 0.5;

    public static final double GEARING_RATIO = 1d; // temp
    public static final Mass WEIGHT = Pound.of(20); // temp
    public static final Distance LENGTH = Inches.of(0.5); // temp
    public static final Distance RADIUS = Inches.of(0.4); // temp
    public static final Distance MIN_HEIGHT = Inches.of(1); // temp
    public static final Distance MAX_HEIGHT = Inches.of(12); // temp
    public static final Distance START_HEIGHT  = Inches.of(3); // temp
    public static final double  STANDARD_DEVIATION = 0d; // temp
}
