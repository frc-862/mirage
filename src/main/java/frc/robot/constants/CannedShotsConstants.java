package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class CannedShotsConstants {
    public enum CannedShot {
        HUB,
        SQUARE_THING,
        CORNER_BOTTOM,
        CORNER_TOP,
        TUNNEL_ENTRANCE_BOTTOM,
        TUNNEL_ENTRANCE_TOP,
    }

    public record ShotData(
        Translation2d target,
        Translation2d shotLocation,
        Angle hoodAngle,
        AngularVelocity shooterSpeed
    ) {
        public Pose2d shotPose() {
            return new Pose2d(shotLocation, target.minus(shotLocation).getAngle());
        }
    }

    public static final HashMap<CannedShot, ShotData> SHOTS = new HashMap<CannedShot, ShotData>() {
        {
            put(CannedShot.HUB, new ShotData(
                new Translation2d(Meters.of(4.62), Meters.of(4)),
                new Translation2d(Meters.of(3), Meters.of(4)),
                Degrees.of(70),
                RotationsPerSecond.of(10)
            )); //placeholder value
            put(CannedShot.SQUARE_THING, new ShotData(
                new Translation2d(Meters.of(0), Meters.of(0)),
                new Translation2d(Meters.of(1.29), Meters.of(6)),
                Degrees.of(0),
                RotationsPerSecond.of(10)
            )); //placeholder value
            put(CannedShot.CORNER_BOTTOM, new ShotData(
                new Translation2d(Meters.of(0), Meters.of(0)),
                new Translation2d(Meters.of(0.5), Meters.of(0.5)),
                Degrees.of(0),
                RotationsPerSecond.of(10)
            )); //placeholder value
            put(CannedShot.CORNER_TOP, new ShotData(
                new Translation2d(Meters.of(0), Meters.of(0)),
                new Translation2d(Meters.of(0.5), Meters.of(7.5)),
                Degrees.of(0),
                RotationsPerSecond.of(10)
            )); //placeholder value
            put(CannedShot.TUNNEL_ENTRANCE_BOTTOM, new ShotData(
                new Translation2d(Meters.of(0), Meters.of(0)),
                new Translation2d(Meters.of(0), Meters.of(0)),
                Degrees.of(0),
                RotationsPerSecond.of(10)
            )); //placeholder value
            put(CannedShot.TUNNEL_ENTRANCE_TOP, new ShotData(
                new Translation2d(Meters.of(0), Meters.of(0)),
                new Translation2d(Meters.of(0), Meters.of(0)),
                Degrees.of(0),
                RotationsPerSecond.of(10)
            )); //placeholder value
        }
    };
}
