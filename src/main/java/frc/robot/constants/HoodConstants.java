// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;


import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.KilogramSquareMeters;


/** Constants for Hood */
public class HoodConstants {
    public static final boolean HOOD_MOTOR_INVERTED = false; // Temp
    public static final double HOOD_MOTOR_STATOR_LIMIT = 40d; // Temp
    public static final boolean HOOD_MOTOR_BRAKE_MODE = false; // Temp

    public static final Angle MIN_ANGLE = Degree.of(-90); // Temp
    public static final Angle MAX_ANGLE = Degree.of(90); // Temp

    public static final double MOTOR_KP = 6.5;
    public static final double MOTOR_KI = 0;
    public static final double MOTOR_KD = 0;
    public static final double MOTOR_KF = 0;
    public static final double MOTOR_KS = 1;
    public static final double MOTOR_KV = 0.18;
    public static final double MOTOR_KA = 0.01;
    public static final double MOTOR_KG = 0;

    public static final double GEARING_RATIO = 74; // Temp

    public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.086); // Temp
    public static final Distance LENGTH = Meter.of(0.18); // Temp

}
