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
    private Shooter shooter;
    private Hood hood;
    private Translation2d target;
    private double distance; 
    private double distanceToTarget;

    /**
     * @param shooter Uses shooter to get the translation of where the shooter is on the field.
     * @param hood Hood Subsystem.
     * @param target Translation 2d for the target.
     * @param distanceToTargetMeters Distance between the hood and the target position to aim at.
     */
    public HoodAim(Shooter shooter, Hood hood, Translation2d target, double distance) {
        this.shooter = shooter;
        this.hood = hood;
        this.target = target;
        this.distance = distance;

        addRequirements(hood);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        //Translation2d shooterTranslation = shooter.getShooterTranslation();

        //distanceToTarget = Meters.of(shooterTranslation.getTranslation().getDistance(target));

        Angle targetAngle = Radians.of(HoodConstants.HOOD_MAP.get(distance));

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
