// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import static edu.wpi.first.units.Units.Degrees;
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
                = inputModulus(optimizeTurretAngle(turretAngle), Turret.TurretConstants.MIN_ANGLE, Turret.TurretConstants.MAX_ANGLE);

        turret.setAngle(wrappedAngle);
    }

    @Override
    public void end(boolean interrupted) {
        // turret.stop();
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
