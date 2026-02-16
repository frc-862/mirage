// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hood.HoodConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HoodAim extends Command {
    /** Creates a new HoodAim. */
    private Swerve drivetrain;
    private Hood hood;
    private Translation2d target;

    /**
     * @param drivetrain Uses drivetrain to get the translation to the target.
     * @param hood Hood Subsystem.
     * @param target Translation 2d for the target.
     */
    public HoodAim(Swerve drivetrain, Hood hood, Translation2d target) {
        this.drivetrain = drivetrain;
        this.hood = hood;
        this.target = target;

        addRequirements(hood);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Translation2d shooterTranslation = drivetrain.getShooterTranslation();

        double distanceToTarget = shooterTranslation.getDistance(target);

        Angle targetAngle = Radians.of(HoodConstants.HOOD_MAP.get(distanceToTarget));

        hood.setPosition(targetAngle);
        

    };

    @Override
    public void end(boolean interrupted) {
        // hood.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
