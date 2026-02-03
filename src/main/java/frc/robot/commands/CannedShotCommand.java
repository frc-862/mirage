package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.LEDConstants.LED_STATES;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;
import frc.util.leds.LEDSubsystem;
import frc.util.shuffleboard.LightningShuffleboard;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;

public class CannedShotCommand {

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

        public static final HashMap<CannedShotsConstants.CannedShot, CannedShotsConstants.ShotData> SHOTS = new HashMap<CannedShotsConstants.CannedShot, CannedShotsConstants.ShotData>() {
            {
                put(CannedShotsConstants.CannedShot.HUB, new ShotData(
                    new Translation2d(Meters.of(4.62), Meters.of(4)),
                    new Translation2d(Meters.of(3), Meters.of(4)),
                    Degrees.of(70),
                    RotationsPerSecond.of(10)
                )); //placeholder value
                put(CannedShotsConstants.CannedShot.SQUARE_THING, new ShotData(
                    new Translation2d(Meters.of(4.62), Meters.of(4)),
                    new Translation2d(Meters.of(1.29), Meters.of(6)),
                    Degrees.of(0),
                    RotationsPerSecond.of(10)
                )); //placeholder value
                put(CannedShotsConstants.CannedShot.CORNER_BOTTOM, new ShotData(
                    new Translation2d(Meters.of(4.62), Meters.of(4)),
                    new Translation2d(Meters.of(0.5), Meters.of(0.5)),
                    Degrees.of(0),
                    RotationsPerSecond.of(10)
                )); //placeholder value
                put(CannedShotsConstants.CannedShot.CORNER_TOP, new ShotData(
                    new Translation2d(Meters.of(4.62), Meters.of(4)),
                    new Translation2d(Meters.of(0.5), Meters.of(7.5)),
                    Degrees.of(0),
                    RotationsPerSecond.of(10)
                )); //placeholder value
                put(CannedShotsConstants.CannedShot.TUNNEL_ENTRANCE_BOTTOM, new ShotData(
                    new Translation2d(Meters.of(4.62), Meters.of(4)),
                    new Translation2d(Meters.of(3.385), Meters.of(0.7)),
                    Degrees.of(0),
                    RotationsPerSecond.of(10)
                )); //placeholder value
                put(CannedShotsConstants.CannedShot.TUNNEL_ENTRANCE_TOP, new ShotData(
                    new Translation2d(Meters.of(4.62), Meters.of(4)),
                    new Translation2d(Meters.of(3.385), Meters.of(7.325)),
                    Degrees.of(0),
                    RotationsPerSecond.of(10)
                )); //placeholder value
            }
        };
    }

    private static CannedShotsConstants.ShotData shotData;
    /**
     * Runs a parallel command to set the shooter motor's power, move hood, and auto align based on the target pose the robot needs to shoot to.
     * Enables an LED State to blink yellow whenever the parallel command is running, and enables an LED State to blink green when the parallel is done running.
     * After the LED blinks green and the parallel command is done running, the shooter motor will idle.
     * @param shooter Shooter subsystem
     * @param hood Hood subsystem
     * @param turret Turret subsystem
     * @param indexer Indexer subsystem
     * @param drivetrain Drivetrain subsystem
     * @param leds LED Subsystem
     * @return the command to run
     */
    public static Command runCannedShot(Shooter shooter, Hood hood, Turret turret, Indexer indexer, Swerve drivetrain, LEDSubsystem leds){
        return Commands.sequence(
            Commands.runOnce(() -> {
                shotData = findClosestShot(drivetrain);
                LightningShuffleboard.setPose2d("CannedShot", "ShotLocation", shotData.shotPose());
            }),
            Commands.parallel(
                new PoseBasedAutoAlign(drivetrain, () -> shotData.shotPose()),
                shooter.shootCommand(() -> shotData.shooterSpeed()),
                hood.hoodCommand(() -> shotData.hoodAngle())
            ).deadlineFor(leds.enableState(LED_STATES.CANNED_SHOT_START.id())),
            indexer.indexCommand(1d).deadlineFor(leds.enableState(LED_STATES.CANNED_SHOT_READY.id()))
        ).handleInterrupt(() -> shooter.stopMotor());
    }

    private static CannedShotsConstants.ShotData findClosestShot(Swerve drivetrain) {
        CannedShotsConstants.ShotData shotData = CannedShotsConstants.SHOTS.get(CannedShotsConstants.CannedShot.HUB); //default shot
        var robotLocation = drivetrain.getPose().getTranslation();
        double currentDistance = Double.POSITIVE_INFINITY;

        for (CannedShotsConstants.ShotData value : CannedShotsConstants.SHOTS.values()){
            double distance = value.shotLocation().minus(robotLocation).getNorm();

            if (distance < currentDistance) {
                currentDistance = distance;
                shotData = value;
            }
        }

        return shotData;
    }
}
