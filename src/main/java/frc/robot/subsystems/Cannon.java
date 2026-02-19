// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Cannon extends SubsystemBase {
    private Shooter shooter;
    private Turret turret;
    private Hood hood;
    // hmm
    private Swerve drivetrain;

    /** Creates a new Cannon. */
    public Cannon(Shooter shooter, Turret turret, Hood hood, Swerve drivetrain) {
        this.shooter = shooter;
        this.turret = turret;
        this.hood = hood;
        this.drivetrain = drivetrain;
    } 

    // ======== GETTERS ========

    /**
     * Gets the translation of the shooter relative to the field
     * @return Returns its position
     */
    public Translation2d getShooterTranslation() {
        return drivetrain.getPose().getTranslation().plus(CannonConstants.SHOOTER_TRANSLATION);
    }

    /**
     * Gets the translation of the target that we want to aim at (all components)
     * @return The transltion
     */
    public Translation2d getTarget() {
        return new Translation2d();
    }

    /**
     * Gets the distance from the robot to the target 
     * @return The distance
     */
    public Distance getTargetDistance() {
        return Meters.of(getTarget().minus(drivetrain.getPose().getTranslation()).getNorm());
    }

    // ======== COMMAND FACTORIES ========

    /**
     * Returns a command to set the hood and shooter values
     * @return The command
     */
    public Command setCannonValues(Angle hoodAngle, AngularVelocity shooterVelocity) {
        return runEnd(() -> {
            shooter.shootCommand(shooterVelocity);
            hood.setPosition(hoodAngle);
        }, () -> shooter.coast());
    }

    /**
     * Returns a command to set the hood and shooter values
     * @return The command
     */
    public Command setCannonValues(Angle turretAngle, Angle hoodAngle, AngularVelocity shooterVelocity) {
        return runEnd(() -> {
            shooter.shootCommand(shooterVelocity);
            hood.setPosition(hoodAngle);
            turret.setAngle(turretAngle);
        }, () -> shooter.coast());
    }


    // ======== CANNON CONSTANTS ========

    public static class CannonConstants {
        public static final Translation2d SHOOTER_TRANSLATION = new Translation2d();

        public static final record candShotValues(Angle hoodAngle){};
    }
}
