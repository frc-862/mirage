// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;
import frc.util.shuffleboard.LightningShuffleboard;

import static frc.util.Units.inputModulus;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurretAim extends Command {

    private Swerve drivetrain;
    private Translation2d target;
    private Turret turret;
    private Distance distanceToTargetMeters;

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
        this.target = findTargetPosition();

        addRequirements(turret);
    }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getPose();

        Translation2d delta = target.minus(robotPose.getTranslation());
        distanceToTargetMeters = Meters.of(delta.getNorm());

        Angle fieldAngle = delta.getAngle().getMeasure();
        Angle turretAngle = fieldAngle.minus(Degree.of(robotPose.getRotation().getDegrees()));

        Angle wrappedAngle
                = inputModulus(
                        turretAngle,
                        Degree.of(-180),
                        Degree.of(180)
                );

        turret.setAngle(wrappedAngle);
        
        LightningShuffleboard.setPose2d("Turret", "target", new Pose2d(target, new Rotation2d()));
        LightningShuffleboard.setPose2d("Turret", "1 robot pose", robotPose);
        LightningShuffleboard.setPose2d("Turret", "2 delta", new Pose2d(delta, new Rotation2d()));
        LightningShuffleboard.setDouble("Turret", "3 field angle", fieldAngle.in(Degree));
        LightningShuffleboard.setDouble("Turret", "4 turret angle", turretAngle.in(Degree));
        LightningShuffleboard.setDouble("Turret", "5 wrapped angle", wrappedAngle.in(Degree));

        LightningShuffleboard.setBool("Turret", "In Zone", isInZone());
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            turret.stop();
        }
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

    /**
     * Finds the optimal target position on the field based on the robot's pose
     * @return Translation2d of the target position
     */
    public Translation2d findTargetPosition() {
        Pose2d robotPose = drivetrain.getPose();
        
        if (isInZone()) {
            return (Swerve.FieldConstants.getTargetData(
                    Swerve.FieldConstants.GOAL_POSITION));
        } else {
            if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Red) {
                if (robotPose.getY() > Swerve.FieldConstants.FIELD_MIDDLE_Y) {
                    return Swerve.FieldConstants.ZONE_POSITION_RED_TOP;
                } else {
                    return Swerve.FieldConstants.ZONE_POSITION_RED_BOTTOM;
                }
            } else {
                if (robotPose.getY() > Swerve.FieldConstants.FIELD_MIDDLE_Y) {
                    return Swerve.FieldConstants.ZONE_POSITION_BLUE_TOP;
                } else {
                    return Swerve.FieldConstants.ZONE_POSITION_BLUE_BOTTOM;
                }
            }
        }
    }

    /**
     * Checks if the robot's pose is within the current alliance's zone
     * @return true if the robot is in the zone, false otherwise
     */
    public boolean isInZone() {
        Pose2d robotPose = drivetrain.getPose();
        
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Red) {
            return (Swerve.FieldConstants.RED_ALLIANCE_RECT.contains(robotPose.getTranslation()));
        } else {
            return (Swerve.FieldConstants.BLUE_ALLIANCE_RECT.contains(robotPose.getTranslation()));
        }
    }
}
