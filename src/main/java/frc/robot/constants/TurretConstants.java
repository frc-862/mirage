package frc.robot.constants;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

public class TurretConstants {
    // Tolerance in degrees
    public static final double TURRET_ANGLE_TOLERANCE = 5;

    public static final Angle MIN_ANGLE = Degree.of(-220);
    public static final Angle MAX_ANGLE = Degree.of(220);

    public static final double MOTOR_KP = 6.5;
    public static final double MOTOR_KI = 0;
    public static final double MOTOR_KD = 0;
    public static final double MOTOR_KF = 0;
    public static final double MOTOR_KS = 1;
    public static final double MOTOR_KV = 0.18;
    public static final double MOTOR_KA = 0.01;
    public static final double MOTOR_KG = 0;

    public static final double ROTOR_TO_ENCODER_RATIO = 74;
    public static final double ENCODER_TO_MECHANISM_RATIO = 1d;

    public static final double turretOffset = -0.227;

    public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.086);
    public static final Distance LENGTH = Meter.of(0.18);

    public static final int TURRET_MOTOR_ID = 0;
    public static final String TURRET_CAN_BUS = "canivore";
    public static final boolean TURRET_MOTOR_INVERTED = false;
    public static final int TURRET_STATOR_LIMIT = 0;
    public static final boolean TURRET_BRAKE = false;

    public static final int TURRET_ENCODER_ID = 0;
}
