// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;
import static frc.util.Units.inputModulus;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurretAim extends Command {

    private Swerve drivetrain;
    private Translation2d target;
    private Turret turret;
    private Distance distanceToTargetMeters;

    private Pose2d pose = new Pose2d(0, 0, Rotation2d.fromDegrees(0)); //Temp
    private Rectangle2d redAllianceZone = new Rectangle2d(pose, 1, 2); //Temp
    private Rectangle2d neutralZone = new Rectangle2d(pose, 2, 1); //Temp
    private Rectangle2d blueallianceZone = new Rectangle2d(pose, 2, 2); //Temp

    /**
     * @param drivetrain drivetrain from the Swerve class to get robot pose
     * @param turret a turret from the Turret subsytem
     * @param target Translation2d for the target
     */
    public TurretAim(Swerve drivetrain, Turret turret, Translation2d target) {
        this.drivetrain = drivetrain;
        this.turret = turret;
        this.target = target;

        addRequirements(turret);
    }

    /**
     * @param drivetrain drivetrain from the Swerve class to get robot pose
     * @param turret a turret from the Turret subsytem
     */
    public TurretAim(Swerve drivetrain, Turret turret) {
        this.drivetrain = drivetrain;
        this.turret = turret;
        this.target = null;

        addRequirements(turret);
    }

    @Override
    public void execute() {

        Pose2d robotPose = drivetrain.getPose();
        Translation2d currentTarget;

        if (target != null) {
            currentTarget = target;
        } else {
            if (redAllianceZone.contains(robotPose.getTranslation())) {
                currentTarget = Swerve.FieldConstants.getTargetData(
                        Swerve.FieldConstants.GOAL_POSITION);
            } else if (neutralZone.contains(robotPose.getTranslation())) {
                currentTarget = Swerve.FieldConstants.getTargetData(
                        Swerve.FieldConstants.DEPOT_POSITION);
            } else {
                currentTarget = Swerve.FieldConstants.getTargetData(
                        Swerve.FieldConstants.DEPOT_POSITION);
            }
        }

        Translation2d delta = currentTarget.minus(robotPose.getTranslation());
        distanceToTargetMeters = Meters.of(delta.getNorm());

        Angle fieldAngle = delta.getAngle().getMeasure();
        Angle turretAngle
                = fieldAngle.minus(Degree.of(robotPose.getRotation().getDegrees()));

        Angle wrappedAngle
                = inputModulus(
                        turretAngle,
                        Turret.TurretConstants.MIN_ANGLE,
                        Turret.TurretConstants.MAX_ANGLE
                );

        turret.setAngle(wrappedAngle);
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }

    @Override
    public boolean isFinished() {
        return turret.isOnTarget();
    }

    /**
     * gets the distance from current turret angle to target turret angle
     *
     * @return distance to target in meters
     */
    public Distance getDistanceToTargetMeters() {
        return distanceToTargetMeters;
    }
}
