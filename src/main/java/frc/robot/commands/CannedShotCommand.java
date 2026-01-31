package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.CannedShotsConstants;
import frc.robot.constants.CannedShotsConstants.CannedShot;
import frc.robot.constants.CannedShotsConstants.ShotData;
import frc.robot.constants.LEDConstants.LED_STATES;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;
import frc.util.leds.LEDSubsystem;

public class CannedShotCommand{
    /**
     * Runs a parallel command to set the shooter motor's power, move hood, and auto align based on the target pose the robot needs to shoot to.
     * Enables an LED State to blink yellow whenever the parallel command is running, and enables an LED State to blink green when the parallel is done running.
     * After the LED blinks green and the parallel command is done running, the shooter motor will idle.
     * @param shot CannedShot object
     * @param shooter Shooter subsystem
     * @param hood Hood subsystem
     * @param turret Turret subsystem
     * @param indexer Indexer subsystem
     * @param drivetrain Drivetrain subsystem
     * @param leds LED Subsystem
     * @return the command to run
     */
    public static Command runCannedShot(Shooter shooter, Hood hood, Turret turret, Indexer indexer, Swerve drivetrain, LEDSubsystem leds){
        CannedShot shot = CannedShot.HUB;
        ShotData shotData = CannedShotsConstants.SHOTS.get(shot);
        var robotLocation = drivetrain.getPose().getTranslation();
        double currentDistance = Double.POSITIVE_INFINITY;

        for (Map.Entry<CannedShot, ShotData> entry : CannedShotsConstants.SHOTS.entrySet()){
            CannedShot key = entry.getKey();
            ShotData value = entry.getValue();

            double distance = value.shotLocation().minus(robotLocation).getNorm();

            if (distance < currentDistance) {
                currentDistance = distance;
                shot = key;
                shotData = value;
            }
        }

        return Commands.sequence(
            Commands.parallel(
                new PoseBasedAutoAlign(drivetrain, shotData.shotPose()),
                shooter.runOnce(() -> shooter.setVelocity(shotData.shooterSpeed())).until(shooter::velocityOnTarget),
                new MoveHood(hood, shotData.hoodAngle())
            ).deadlineFor(leds.enableState(LED_STATES.CANNED_SHOT_START.id())),
            new Index(indexer, 1).deadlineFor(leds.enableState(LED_STATES.CANNED_SHOT_READY.id()))
        ).handleInterrupt(() -> shooter.stopMotor());
    }
}
