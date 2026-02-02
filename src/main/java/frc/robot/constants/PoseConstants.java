package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class PoseConstants {
    public static final double DRIVE_P = 1.5d;
    public static final double DRIVE_I = 0;
    public static final double DRIVE_D = 0.08;
    public static final Distance DRIVE_TOLERANCE = Meters.of(0.05);
    public static final double DRIVE_KS = 0;

    public static final double ROT_P = 0.03;
    public static final double ROT_I = 0;
    public static final double ROT_D = 0;
    public static final Angle ROT_TOLERANCE = Degrees.of(1.5);
    public static final double ROT_KS = 0; // 0.01 NOT APPLIED
}
