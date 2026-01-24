package frc.robot.constants;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meter;

import java.util.HashMap;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;

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

    public enum Canned_Shot {
        HUB,
        SQUARE_THING,
        CORNER_BOTTOM,
        CORNER_TOP,
        TUNNEL_ENTRANCE_BOTTOM,
        TUNNEL_ENTRANCE_TOP,
    }

    public static final HashMap<Canned_Shot, Double> SHOOTER_RPM_MAP = new HashMap<Canned_Shot, Double>() {
        {
            put(Canned_Shot.HUB, 1d);                   //placeholder value
            put(Canned_Shot.SQUARE_THING, 1d);          //placeholder value
            put(Canned_Shot.CORNER_BOTTOM, 1d);         //placeholder value
            put(Canned_Shot.CORNER_TOP, 1d);            //placeholder value
            put(Canned_Shot.TUNNEL_ENTRANCE_BOTTOM, 1d);//placeholder value
            put(Canned_Shot.TUNNEL_ENTRANCE_TOP, 1d);   //placeholder value
        }
    };

    public static final HashMap<Canned_Shot, Double> HOOD_ANGlE_MAP = new HashMap<Canned_Shot, Double>() {
        {
            put(Canned_Shot.HUB, 1d);                   //placeholder value
            put(Canned_Shot.SQUARE_THING, 1d);          //placeholder value
            put(Canned_Shot.CORNER_BOTTOM, 1d);         //placeholder value
            put(Canned_Shot.CORNER_TOP, 1d);            //placeholder value
            put(Canned_Shot.TUNNEL_ENTRANCE_BOTTOM, 1d);//placeholder value
            put(Canned_Shot.TUNNEL_ENTRANCE_TOP, 1d);   //placeholder value
        }
    };
}
