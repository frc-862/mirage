package frc.robot.constants;

import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.Degree;

public class TurretConstants {
    // Tolerance in degrees
    public static final double TURRET_ANGLE_TOLERANCE = 5;

    public static final Angle MIN_ANGLE = Degree.of(-220);
    public static final Angle MAX_ANGLE = Degree.of(220);

    public static final double MOTOR_KP = 0;
    public static final double MOTOR_KI = 0;
    public static final double MOTOR_KD = 0;
    public static final double MOTOR_KF = 0;
    public static final double MOTOR_KS = 0;
    public static final double MOTOR_KV = 0;
    public static final double MOTOR_KA = 0;
    public static final double MOTOR_KG = 0;

    public static final double ROTOR_TO_ENCODER_RATIO = 0;
    public static final double ENCODER_TO_MECHANISM_RATIO = 0;

    public static final double wristOffset = 0;
}
