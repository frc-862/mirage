// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Target;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.util.AllianceHelpers;
import frc.util.shuffleboard.LightningShuffleboard;

public class Cannon extends SubsystemBase {
    // ======== CANNON CONSTANTS ========

    public class CannonConstants { 
        public static final Distance SMART_SHOOT_MIN_DISTANCE = Meters.of(1.902d);
        public static final Translation2d SHOOTER_TRANSLATION = new Translation2d(Inches.of(3.275), Inches.of(-3.275));
        public static final Distance SHOOTER_HEIGHT = Inches.of(18);

        public record CandShot(Angle turretAngle, Angle hoodAngle, AngularVelocity shooterVelocity){};

        // TODO: ADD CAND SHOT VALUES HERE TO CREATE CAND SHOTS USING THE NEW BETTER METHOD :)
        public static final CandShot LEFT_SHOT = new CandShot(Degrees.of(0),Degrees.of(0), RadiansPerSecond.of(0));//Temp
        public static final CandShot RIGHT_SHOT = new CandShot(Degrees.of(0),Degrees.of(0), RadiansPerSecond.of(0));//Temp
        public static final CandShot MIDDLE_SHOT = new CandShot(Degrees.of(0),Degrees.of(0), RadiansPerSecond.of(0));//Temp
    }

    // Subsystems
    private Shooter shooter;
    private Turret turret;
    private Hood hood;
    private Swerve drivetrain;
    private Indexer indexer;

    // Target storage
    private Translation2d storedTarget;

    private DoubleArrayLogEntry targetPositionLog;
    private DoubleLogEntry distToTargetLog;

    /**
     * Creates a new cannon
     * @param shooter shooter
     * @param turret turret
     * @param hood hood 
     * @param indexer indexer
     * @param drivetrain serve
     */
    public Cannon(Shooter shooter, Turret turret, Hood hood, Swerve drivetrain, Indexer indexer) {
        this.shooter = shooter;
        this.turret = turret;
        this.hood = hood;
        this.drivetrain = drivetrain;

        this.indexer = indexer;

        initLogging();
    }

    private void initLogging() {
        DataLog log = DataLogManager.getLog();

        targetPositionLog = new DoubleArrayLogEntry(log, "/Cannon/TargetPosition");
        distToTargetLog = new DoubleLogEntry(log, "/Cannon/DistanceToTarget");
    }

    @Override
    public void periodic() {
        // Code to calculate target position
        Pose2d robotPose = drivetrain.getPose();
        
        boolean isTop = robotPose.getY() > FieldConstants.FIELD_MIDDLE_Y;

        if (drivetrain.isInZone()) {
            storedTarget =  FieldConstants.getTargetData(FieldConstants.GOAL_POSITION);
        } else if (AllianceHelpers.isBlueAlliance()) {
            storedTarget =  isTop ? FieldConstants.ZONE_POSITION_BLUE_TOP : FieldConstants.ZONE_POSITION_BLUE_BOTTOM;
        } else {
            storedTarget =  isTop ? FieldConstants.ZONE_POSITION_RED_TOP : FieldConstants.ZONE_POSITION_RED_BOTTOM;
        }

        updateLogging();
    }

    private void updateLogging() {
        targetPositionLog.append(new double[]{getTarget().getX(), getTarget().getY()});
        distToTargetLog.append(getTargetDistance().in(Meters));

        if(!DriverStation.isFMSAttached() || Robot.isSimulation()) {
            LightningShuffleboard.setTranslation2d("Cannon", "Target Position", getTarget());
            LightningShuffleboard.setDouble("Cannon", "Distance To Target", getTargetDistance().in(Meters));
        }
    }

    /**
     * Gets the translation of the shooter relative to the field
     * @return Returns its position
     */
    public Translation2d getShooterTranslation() {
        return drivetrain.getPose().getTranslation().plus(CannonConstants.SHOOTER_TRANSLATION.rotateBy(drivetrain.getPose().getRotation()));
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
        return Meters.of(getTarget().minus(getShooterTranslation()).getNorm());
    }

    /**
     * Returns a command to set the hood and shooter values
     * @param hoodAngle The angle of the hood to set at
     * @param shooterVelocity The velocity to set the shooter to
     * @return The command
     */
    public Command createCannonCommand(Angle hoodAngle, AngularVelocity shooterVelocity) {
        return new ParallelCommandGroup(
            shooter.shootCommand(shooterVelocity), 
            hood.setPositionCommand(hoodAngle)
        );     
    }

    /**
     * Returns a command to set the hood and shooter values
     * @param turretAngle the angle to set the turret to
     * @param hoodAngle the angle to set the hood to
     * @param shooterVelocity the velocity to set the shooter to 
     * @return The command
     */
    public Command createCannonCommand(Angle turretAngle, Angle hoodAngle, AngularVelocity shooterVelocity) {
        return new ParallelCommandGroup(
            shooter.shootCommand(shooterVelocity), 
            turret.setAngleCommand(turretAngle), 
            hood.setPositionCommand(hoodAngle)
        );
    }

    /**
     * Creates a cand shot command that uses only hood and shooter
     * @param value The cand shot value
     * @return The command
     */
    public Command createCandShotCommand(CannonConstants.CandShot value) {
        return createCannonCommand(Degrees.of(0), value.hoodAngle, value.shooterVelocity);
    }

    /**
     * Creates a cand shot command that also uses turret
     * @param value The cand shot to use
     * @return The command
     */
    public Command createTurretCandShotCommand(CannonConstants.CandShot value) {
        return createCannonCommand(value.turretAngle, value.hoodAngle, value.shooterVelocity);
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
        return shooter.runShootCommand(() -> Shooter.ShooterConstants.VELOCITY_MAP.get(getTargetDistance()))
        .alongWith(new SequentialCommandGroup(
            new WaitUntilCommand(() -> turret.isOnTarget() && hood.isOnTarget() && shooter.isOnTarget() && !isNearHub()),
            indexer.indexCommand(Indexer.IndexerConstants.SPINDEXDER_POWER, Indexer.IndexerConstants.TRANSFER_POWER))
        ).finallyDo((end) -> {
            shooter.setPower(Shooter.ShooterConstants.COAST_DC);
            indexer.stop();
        });
    }

    /**
     * finds the distance between shooter and hub and calculates if it is nearby.
     * @return if the distance of shooter on the field and the hub is less than 1
     */
    public boolean isNearHub(){
          Distance distance = Meters.of(this.getShooterTranslation().getDistance(FieldConstants.getTargetData(FieldConstants.GOAL_POSITION)));
          return distance.lt(CannonConstants.SMART_SHOOT_MIN_DISTANCE);
    }
}