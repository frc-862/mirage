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
    private final Swerve swerve;
    private final Hood hood;
    private final Turret turret;
    private final Indexer indexer;

    private Translation2d target = new Translation2d(4.625594, 4.034536);

    private Pose2d pose = new Pose2d(0, 0, Rotation2d.fromDegrees(0)); //Temp
    private Rectangle2d redAllianceZone = new Rectangle2d(pose, 0, 0); //Temp
    private Rectangle2d neutralZone = new Rectangle2d(pose, 1, 1); //Temp
    private Rectangle2d blueallianceZone = new Rectangle2d(pose, 2, 2); //Temp

    /**
     * Point the shooter and angle the hood so the fuel shot reaches the hub
     * @param shooter the shooter subsystem
     * @param swerve swerve to get robot pose
     * @param hood the hood susbsystem
     * @param turret the turret subsystem
     * @param indexer the indexer subs
     */
    public ShooterAim(Shooter shooter, Swerve swerve, Hood hood, Turret turret, Indexer indexer){
        this.shooter = shooter;
        this.swerve = swerve;
        this.hood = hood;
        this.turret = turret;
        this.indexer = indexer;

        addRequirements(shooter, hood, indexer);
    }

    @Override
    public void execute() {
        if (turret.isOnTarget() && hood.isOnTarget() && shooter.isOnTarget()) {
            indexer.setSpindexerPower(Indexer.IndexerConstants.SPINDEXDER_POWER); //TEMP
            indexer.setTransferPower(Indexer.IndexerConstants.TRANSFER_POWER);
        }

        Pose2d robotPose = swerve.getPose();

        // if(redAllianceZone.contains(robotPose.getTranslation())){
        //     target = Swerve.FieldConstants.getTargetData(Swerve.FieldConstants.GOAL_POSITION);
        // }
        // else if (neutralZone.contains(robotPose.getTranslation())) {
        //     target = Swerve.FieldConstants.getTargetData(Swerve.FieldConstants.DEPOT_POSITION);
        // }
        // else if(blueallianceZone.contains(robotPose.getTranslation())){
        //     target = Swerve.FieldConstants.getTargetData(Swerve.FieldConstants.DEPOT_POSITION);
        // }

        Distance distance = Meters.of(robotPose.getTranslation().getDistance(target));

        AngularVelocity shooterVelocity = ShooterConstants.VELOCITY_MAP.get(distance);
        Angle hoodAngle = HoodConstants.HOOD_MAP.get(distance);

        hood.setPosition(hoodAngle);
        shooter.setVelocity(shooterVelocity);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        hood.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
