// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.lang.annotation.Target;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.HoodConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;

public class ShooterAim extends Command {
    private final Shooter shooter;
    private final Swerve swerve;
    private final Hood hood;
    private final Turret turret;
    private final Indexer indexer;

    private Translation2d target;

    private Pose2d pose = new Pose2d(2.312797, 0, Rotation2d.fromDegrees(0));
    private Rectangle2d redAllianceZone = new Rectangle2d(pose, 0, 0);

    /**
     * Point the shooter and angle the hood so the fuel shot reaches the hub
     * @param shooter the shooter subsystem
     * @param swerve swerve to get robot pose
     * @param hood the hood susbsystem
     * @param turret the turret subsystem
     */
    public ShooterAim(Shooter shooter, Swerve swerve, Hood hood, Turret turret, Indexer indexer){
        this.shooter = shooter;
        this.swerve = swerve;
        this.hood = hood;
        this.turret = turret;
        this.indexer = indexer;

        addRequirements(shooter, hood, turret);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (turret.isOnTarget() & hood.isOnTarget() & shooter.isOnTarget()) {
            indexer.setPower(0); //TEMP
        }

        Pose2d robotPose = swerve.getPose();
        double distanceMeters = robotPose.getTranslation().getDistance(target);

        if(redAllianceZone.contains(robotPose.getTranslation())){
            // target = FieldConstants.getTargetData(FieldConstants.GOAL_POSITION);
        }


        AngularVelocity shooterVelocity = RotationsPerSecond.of(ShooterConstants.VELOCITY_MAP.get(distanceMeters));
        Angle hoodAngle = Degrees.of(HoodConstants.HOOD_MAP.get(distanceMeters));

        hood.setPosition(hoodAngle);
        shooter.setVelocity(shooterVelocity);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.stopMotor();
        hood.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
