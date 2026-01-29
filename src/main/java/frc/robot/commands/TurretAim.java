// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;

import static frc.util.Units.inputModulus;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurretAim extends Command {
    private Swerve drivetrain;
    private Translation2d target;
    private Turret turret;
    private Distance distanceToTargetMeters;

    /**
     * @param drivetrain drivetrain from the Swerve class to get robot pose
     * @param turret a turret from the Turret subsytem
     * @param target a Translation2d representing where the turret should aim at
     */
    public TurretAim(Swerve drivetrain, Turret turret, Translation2d target) {
        this.drivetrain = drivetrain;
        this.turret = turret;
        this.target = target;

        addRequirements(turret);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getPose();

        Translation2d delta = target.minus(robotPose.getTranslation());

        /*
        * Get the distance from the robot to the target shot via the norm of the
        * translation
        */
        distanceToTargetMeters = Meters.of(delta.getNorm());

        /*
        * Well add values if we have to based on however the angles acually get
        * calculated
        */
        Angle fieldAngle = delta.getAngle().getMeasure();
        Angle turretAngle = fieldAngle.minus(Degree.of(robotPose.getRotation().getDegrees()));

        // Adjust the angle based on the minimum and maximum angles of the turret
        Angle wrappedAngle = inputModulus(turretAngle, TurretConstants.MIN_ANGLE, TurretConstants.MAX_ANGLE);

        turret.setAngle(wrappedAngle);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return turret.isOnTarget();
    }

    /**
     * gets the distance from current turret angle to target turret angle
     * @return distance to target in meters
     */
    public Distance getDistanceToTargetMeters() {
        return distanceToTargetMeters;
    }
}
