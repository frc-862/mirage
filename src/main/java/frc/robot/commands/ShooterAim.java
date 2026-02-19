// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hood.HoodConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;

public class ShooterAim extends Command {
    private final Shooter shooter;
    private final Swerve drivetrain;
    private final Hood hood;
    private final Turret turret;
    private final Indexer indexer;


    /**
     * Point the shooter and angle the hood so the fuel shot reaches the hub
     * @param shooter the shooter subsystem
     * @param swerve swerve to get robot pose
     * @param hood the hood susbsystem
     * @param turret the turret subsystem
     * @param indexer the indexer subs
     */
    public ShooterAim(Shooter shooter, Swerve drivetrain, Hood hood, Turret turret, Indexer indexer){
        this.shooter = shooter;
        this.drivetrain = drivetrain;
        this.hood = hood;
        this.turret = turret;
        this.indexer = indexer;

        addRequirements(shooter, indexer);
    }

    @Override
    public void execute() {
        if (turret.isOnTarget() & hood.isOnTarget() & shooter.isOnTarget()) {
            indexer.setSpindexerPower(Indexer.IndexerConstants.SPINDEXDER_POWER); //TEMP
            indexer.setTransferPower(Indexer.IndexerConstants.TRANSFER_POWER);
        }

        Distance distance = Meters.of(drivetrain.getShooterTranslation().getDistance(drivetrain.getTargetPosition()));
        AngularVelocity shooterVelocity = ShooterConstants.VELOCITY_MAP.get(distance);

        shooter.setVelocity(shooterVelocity);
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished() {
        return false;
    }
}
