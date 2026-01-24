package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public class CollectorConstants {
    // collector intake motor
    public static final boolean INTAKE_MOTOR_INVERTED = false; // Temp
    public static final double INTAKE_MOTOR_STATOR_LIMIT = 40d; // Temp
    public static final boolean INTAKE_MOTOR_BRAKE_MODE = true; // Temp
    public static final double INTAKE_POWER = 1d;

    public static final double INTAKE_SIM_kV = 0.24;
    public static final double INTAKE_SIM_kA = 0.8;

    // collector pivot motor config
    public static final double PIVOT_KP = 0.3d; // temp
    public static final double PIVOT_KI = 0d; // temp
    public static final double PIVOT_KD = 0.02d; // temp
    public static final double PIVOT_KS = 0; // temp
    public static final double PIVOT_KV = 0;
    public static final double PIVOT_KA = 0;
    public static final double PIVOT_KG = 0.3d; // temp

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
}
