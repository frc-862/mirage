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
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radian;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurretAim extends Command {
    private Swerve drivetrain;
    private Translation2d target;
    private Turret turret;
    private double distanceToTargetMeters;

    /** Creates a new TurretAim. */
    public TurretAim(Swerve drivetrain, Turret turret, Translation2d target) {
        this.drivetrain = drivetrain;
        this.target = target;
        this.turret = turret;

        addRequirements(turret);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Get our robot pose
        Pose2d robotPose = drivetrain.getPose();

        // Get the translation assuming the robot at (0, 0)
        Translation2d delta = target.minus(robotPose.getTranslation());

        // Get the distance from the robot to the target shot via the norm of the
        // translation
        distanceToTargetMeters = delta.getNorm();

        // Well add values if we have to based on however the angles acually get
        // calculated
        Angle fieldAngle = delta.getAngle().getMeasure();
        Angle turretAngle = fieldAngle.minus(Degree.of(robotPose.getRotation().getDegrees()));

        // Adjust the angle based on the minimum and maximum angles of the turret
        double wrappedAngle = MathUtil.inputModulus(turretAngle.in(Degree), TurretConstants.MIN_ANGLE.in(Degree),
                TurretConstants.MAX_ANGLE.in(Degree));

        // Set the turret angle based off our robot angle as well
        turret.setAngle(Degree.of(robotPose.getRotation().getDegrees()).minus(Degree.of(wrappedAngle)));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // if Math.abs(turretTargetAngle - turret.getAngle()) > someTolerance
        return turret.isOnTarget();
    }

    public Distance getDistanceToTargetMeters() {
        return Meters.of(distanceToTargetMeters);
    }
}
