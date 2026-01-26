package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public class CollectorConstants {
    // collector motor for the rollers
    public static final boolean COLLECTOR_MOTOR_INVERTED = false; // temp
    public static final double COLLECTOR_MOTOR_STATOR_LIMIT = 40d; // temp
    public static final boolean COLLECTOR_MOTOR_BRAKE = true; // temp
    public static final double COLLECT_POWER = 1d;

    public static final double COLLECTOR_SIM_kV = 0.24; // temp
    public static final double COLLECTOR_SIM_kA = 0.8; // temp

    // collector pivot motor config
    public static final double PIVOT_KP = 0.3d; // temp
    public static final double PIVOT_KI = 0d; // temp
    public static final double PIVOT_KD = 0.02d; // temp
    public static final double PIVOT_KS = 0; // temp
    public static final double PIVOT_KV = 0; // temp
    public static final double PIVOT_KA = 0; // temp
    public static final double PIVOT_KG = 0.3d; // temp

    // collector pivot
    public static final boolean PIVOT_INVERTED = false; // temp
    public static final double PIVOT_STATOR_LIMIT = 40d; // temp
    public static final boolean PIVOT_BRAKE_MODE = true; // temp
    public static final double PIVOT_OFFSET = -0.227; // temp
    public static final double ROTOR_TO_ENCODER_RATIO = 74; // temp
    public static final double ENCODER_TO_MECHANISM_RATIO = 1d; // temp
    public static final Angle MIN_ANGLE = Degrees.of(-85); // temp
    public static final Angle MAX_ANGLE = Degrees.of(85); // temp
    public static final Angle TOLERANCE = Degrees.of(5); // temp

    // Sim
    public static final Distance WIDTH = Inches.of(27); // temp
    public static final Distance LENGTH_EXTENDED = Inches.of(7); // temp
    public static final int ROBOT_FUEL_CAPACITY = 50; // temp
    public static final AngularVelocity SIM_COLLECTING_THRESHOLD = RotationsPerSecond.of(1); // temp
}
