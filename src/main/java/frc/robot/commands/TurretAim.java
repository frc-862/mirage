// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;
import frc.util.shuffleboard.LightningShuffleboard;

import static frc.util.Units.inputModulus;
import frc.util.AllianceHelpers;

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
        this.target = drivetrain.findTargetPosition();

        addRequirements(turret);
    }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getPose();

        Translation2d delta = target.minus(robotPose.getTranslation());
        distanceToTargetMeters = Meters.of(delta.getNorm());

        Angle fieldAngle = delta.getAngle().getMeasure();

        Angle turretAngle
                = fieldAngle.minus(robotPose.getRotation().getMeasure());

        Angle wrappedAngle
                = inputModulus(optimizeTurretAngle(turretAngle), Degree.of(-180), Degree.of(180));

        turret.setAngle(wrappedAngle);
        
        LightningShuffleboard.setPose2d("Turret", "target", new Pose2d(target, new Rotation2d()));
        LightningShuffleboard.setPose2d("Turret", "1 robot pose", robotPose);
        LightningShuffleboard.setPose2d("Turret", "2 delta", new Pose2d(delta, new Rotation2d()));
        LightningShuffleboard.setDouble("Turret", "3 field angle", fieldAngle.in(Degree));
        LightningShuffleboard.setDouble("Turret", "4 turret angle", turretAngle.in(Degree));
        LightningShuffleboard.setDouble("Turret", "5 wrapped angle", wrappedAngle.in(Degree));

        LightningShuffleboard.setBool("Turret", "In Zone", drivetrain.isInZone());
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
     * Extend angle past 180 / -180 if possible while remaining within -220 /
     * 220
     *
     * @param desired the angle between -180 and 180 calculated for the turret
     * @return the angle optimized between -220 and 220
     */
    private Angle optimizeTurretAngle(Angle desired) {
        double targetDeg = desired.in(Degrees);
        double current = turret.getAngle().in(Degrees);

        double error = targetDeg - current;

        if (error > 180) {
            targetDeg -= 360;
        } else if (error < -180) {
            targetDeg += 360;
        }

        return Degrees.of(targetDeg);
    }
}
