// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Target;
import frc.util.AllianceHelpers;

public class Cannon extends SubsystemBase {
    // Subsystems
    private final Shooter shooter;
    private final Turret turret;
    private final Hood hood;
    private final Swerve drivetrain;

    // Target storage
    private Translation2d storedTarget;

    /** Creates a new Cannon. */
    public Cannon(Shooter shooter, Turret turret, Hood hood, Swerve drivetrain) {
        this.shooter = shooter;
        this.turret = turret;
        this.hood = hood;
        this.drivetrain = drivetrain;
    } 

    @Override
    public void periodic() {
        // Code to calculate target position
        Pose2d robotPose = drivetrain.getPose();

        if (drivetrain.isInZone()) {
            storedTarget =  FieldConstants.getTargetData(FieldConstants.GOAL_POSITION);
        }
        
        boolean isTop = robotPose.getY() > FieldConstants.FIELD_MIDDLE_Y;

        if (AllianceHelpers.isBlueAlliance()) {
            storedTarget =  isTop ? FieldConstants.ZONE_POSITION_BLUE_TOP : FieldConstants.ZONE_POSITION_BLUE_BOTTOM;
        } else {
            storedTarget =  isTop ? FieldConstants.ZONE_POSITION_RED_TOP : FieldConstants.ZONE_POSITION_RED_BOTTOM;
        }
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
        return storedTarget;
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

    /**
     * Sets the hood angle at a predetermined target by cannon
     * @return The command
     */
    public Command hoodAim() {
        return hood.hoodAim(this);
    }

    /**
     * Sets the hoods angle based on a passed in target
     * @param target The target to use
     * @return the command
     */
    public Command hoodAim(Target target) {
        return hood.hoodAim(this, target);
    }

    /**
     * Aims the turret at a predetermined target
     * @return The command
     */
    public Command turretAim() {
        return turret.turretAim(this);
    }

    /**
     * Aims the turret at a passed in target
     * @param target The target to aim at
     * @return The command
     */
    public Command turretAim(Target target) {
        return turret.turretAim(target);
    }

    /**
     * Remodel of shooter aim-- automatically decides when to shoot 
     * @return The command to run
     */
    public Command smartShoot() {
        return run(() -> {
            
        });
    }


    // ======== CANNON CONSTANTS ========

    public static class CannonConstants {
        public static final Translation2d SHOOTER_TRANSLATION = new Translation2d();

        public static final record candShotValues(Angle hoodAngle){};
    }
}
