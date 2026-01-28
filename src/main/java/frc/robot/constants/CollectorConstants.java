package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;

public class CollectorConstants {
    // collector motor for the rollers
    public static final boolean COLLECTOR_MOTOR_INVERTED = false; // temp
    public static final double COLLECTOR_MOTOR_STATOR_LIMIT = 40d; // temp
    public static final boolean COLLECTOR_MOTOR_BRAKE = true; // temp
    public static final double COLLECT_POWER = 1d;

    public static final double COLLECTOR_SIM_kV = 0.24; // temp
    public static final double COLLECTOR_SIM_kA = 0.8; // temp

    // collector pivot motor config
    public static final double PIVOT_KP = 3d; // temp
    public static final double PIVOT_KI = 0d; // temp
    public static final double PIVOT_KD = 1d; // temp
    public static final double PIVOT_KS = 0; // temp
    public static final double PIVOT_KV = 0; // temp
    public static final double PIVOT_KA = 0; // temp
    public static final double PIVOT_KG = 0d; // temp

    // collector pivot
    public static final boolean PIVOT_INVERTED = false; // Temp
    public static final double PIVOT_STATOR_LIMIT = 40d; // Temp
    public static final boolean PIVOT_BRAKE_MODE = true; // Temp
    public static final double PIVOT_OFFSET = -0.227; // Temp
    public static final double ROTOR_TO_ENCODER_RATIO = 74; // Temp
    public static final double ENCODER_TO_MECHANISM_RATIO = 1d; // Temp
    public static final Angle MIN_ANGLE = Degrees.of(-85); // Temp
    public static final Angle MAX_ANGLE = Degrees.of(85); // Temp
    public static final Angle TOLERANCE = Degrees.of(5); // Temp

    public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.1); //Temp
    public static final Distance LENGTH = Inches.of(6);
}
