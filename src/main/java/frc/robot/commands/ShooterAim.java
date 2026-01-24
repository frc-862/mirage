// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Swerve;

public class ShooterAim extends Command {

    private final Flywheel shooter;
    private final Swerve swerve;
    private final Translation2d target;

    // Input is in meters, output is in rotations per second
    private final InterpolatingDoubleTreeMap VELOCITY_MAP = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(2d, 20d),
        Map.entry(4d, 40d),
        Map.entry(6d, 60d)
    );

    public ShooterAim(Flywheel shooter, Swerve swerve, Translation2d target) {
        this.shooter = shooter;
        this.swerve = swerve;
        this.target = target;
        addRequirements(shooter);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Pose2d robotPose = swerve.getPose();
        double distanceMeters = robotPose.getTranslation().getDistance(target);

        AngularVelocity shooterVelocity = RotationsPerSecond.of(VELOCITY_MAP.get(distanceMeters));

        shooter.setVelocity(shooterVelocity);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.stopMotor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
