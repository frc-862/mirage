// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Swerve;

public class ShooterAim extends Command {

    private final Flywheel shooter;
    private final Swerve swerve;
    private final Hood hood;
    private final Translation2d target;

    // Input is distance to target in meters, output is shooter speed in rotations per second
    private static final InterpolatingDoubleTreeMap VELOCITY_MAP = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(2d, 20d),
        Map.entry(4d, 40d),
        Map.entry(6d, 60d)
    );

    // Input is distance to target in meters, output is hood angle in degrees
    private static final InterpolatingDoubleTreeMap HOOD_MAP = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(2d, 10d),
        Map.entry(4d, 20d),
        Map.entry(6d, 30d)
    );

    public ShooterAim(Flywheel shooter, Swerve swerve, Hood hood, Translation2d target) {
        this.shooter = shooter;
        this.swerve = swerve;
        this.hood = hood;
        this.target = target;
        addRequirements(shooter, hood);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Pose2d robotPose = swerve.getPose();
        double distanceMeters = robotPose.getTranslation().getDistance(target);

        AngularVelocity shooterVelocity = RotationsPerSecond.of(VELOCITY_MAP.get(distanceMeters));
        Angle hoodAngle = Degrees.of(HOOD_MAP.get(distanceMeters));

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
