package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.CannedShotsConstants;
import frc.robot.constants.CannedShotsConstants.CannedShot;
import frc.robot.constants.LEDConstants.LED_STATES;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;
import frc.util.leds.LEDSubsystem;

public class RunCannedShot extends SequentialCommandGroup {

    public RunCannedShot(CannedShot shot, Shooter shooter, Hood hood, Turret turret, Swerve drivetrain, LEDSubsystem leds){
        var shotData = CannedShotsConstants.SHOTS.get(shot);
        addCommands(
            Commands.sequence(
                Commands.parallel(
                    new PoseBasedAutoAlign(drivetrain, shotData.shotPose()),
                    shooter.runOnce(() -> shooter.setVelocity(shotData.shooterSpeed())).until(shooter::velocityOnTarget),
                    new MoveHood(hood, shotData.hoodAngle())
                ).deadlineFor(leds.enableState(LED_STATES.CANNED_SHOT_START.id())),
                leds.enableState(LED_STATES.CANNED_SHOT_READY.id())
            ).handleInterrupt(() -> shooter.stopMotor())
        );
    }
}
