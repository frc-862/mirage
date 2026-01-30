package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.CannedShotsConstants;
import frc.robot.constants.CannedShotsConstants.CannedShot;
import frc.robot.constants.LEDConstants.LED_STATES;
import frc.robot.subsystems.Hood;
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
     * @param drivetrain Drivetrain subsystem
     * @param leds LED Subsystem
     * @return the command to run
     */
    public static Command runCannedShot(CannedShot shot, Shooter shooter, Hood hood, Turret turret, Swerve drivetrain, LEDSubsystem leds){
        var shotData = CannedShotsConstants.SHOTS.get(shot);
        return Commands.sequence(
            Commands.parallel(
                new PoseBasedAutoAlign(drivetrain, shotData.shotPose()),
                shooter.runOnce(() -> shooter.setVelocity(shotData.shooterSpeed())).until(shooter::velocityOnTarget),
                new MoveHood(hood, shotData.hoodAngle())
            ).deadlineFor(leds.enableState(LED_STATES.CANNED_SHOT_START.id())),
            leds.enableState(LED_STATES.CANNED_SHOT_READY.id())
        ).handleInterrupt(() -> shooter.stopMotor());
    }
}
