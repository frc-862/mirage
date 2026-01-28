package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.CannedShotsConstants;
import frc.robot.constants.CannedShotsConstants.CannedShot;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;

public class RunCannedShot extends SequentialCommandGroup {

    public RunCannedShot(CannedShot shot, Shooter shooter, Hood hood, Turret turret, Swerve drivetrain, Indexer indexer){
        var shotData = CannedShotsConstants.SHOTS.get(shot);
        addCommands(
            Commands.parallel(
                new Shoot(shooter, shotData.shooterSpeed()),
                new TurretAim(drivetrain, turret, shotData.target()),
                new MoveHood(hood, shotData.hoodAngle())
            ),
            new Index(indexer, 1)
        );
    }
}
