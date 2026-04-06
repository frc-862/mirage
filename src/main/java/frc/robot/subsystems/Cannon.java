// Copyright (c) FIRST and other WPILib contributors
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Target;
import frc.robot.subsystems.Indexer.IndexerConstants;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.util.AllianceHelpers;
import frc.util.shuffleboard.LightningShuffleboard;
import frc.util.units.SplineMap;
import frc.util.units.ThunderMap;

public class Cannon extends SubsystemBase {
    // ======== CANNON CONSTANTS ========

    public class CannonConstants { 
        public static final Distance SMART_SHOOT_MIN_DISTANCE = Inches.of(64);
        public static final Translation2d SHOOTER_TRANSLATION = new Translation2d(Inches.of(7.5), Inches.of(-7.5));
        public static final Transform2d SHOOTER_TRANSFORM = new Transform2d(SHOOTER_TRANSLATION, new Rotation2d());
        public static final Distance SHOOTER_HEIGHT = Inches.of(18);

        public record CandShot(Angle turretAngle, Angle hoodAngle, AngularVelocity shooterVelocity){};

        // SplineMap uses cubic spline interpolation instead of linear.
        // This correctly handles non-monotonic TOF data (e.g. high-arc shots
        // at 228" take longer than flatter shots at 262").
        public static final SplineMap<Distance, Time> TIME_OF_FLIGHT_MAP = new SplineMap<Distance, Time>() {{
            // put(Inches.of(18.78*12), Seconds.of(35.0/30.0));
            // put(Inches.of(64), Seconds.of(24.0/30.0));
            // put(Inches.of(142), Seconds.of(0.86));
            put(Inches.of(60), Seconds.of(0.88));
            put(Inches.of(102), Seconds.of(1.0));
            put(Inches.of(144), Seconds.of(1.166));
            put(Inches.of(186), Seconds.of(1.51));
            put(Inches.of(228), Seconds.of(1.4));
            put(Inches.of(262), Seconds.of(1.46));
            put(Inches.of(298), Seconds.of(1.66));
        }};

        public static final int MAX_OTF_ITERATIONS = 15;
        public static final Distance OTF_TOLERANCE = Inches.of(1.5);

        // Under-relaxation factor for the OTF convergence loop.
        // Each iteration blends the new prediction with the previous one:
        //   pose_{n+1} = lerp(pose_n, prediction, RELAXATION)
        //
        // Without relaxation (factor = 1.0), convergence requires |v * dTOF/dd| < 1.
        // With the spline TOF map, slopes can reach ~0.30 s/m, so at 4 m/s the
        // raw contraction factor is 1.2 -- diverges!
        //
        // With relaxation, the effective contraction becomes RELAXATION * |v * dTOF/dd|.
        // At 0.5: worst case is 0.5 * 5.0 * 0.35 = 0.875 -- converges at any FRC speed.
        // Trade-off: lower = safer convergence but needs more iterations.
        public static final double OTF_RELAXATION = 0.5;
      
        public static final CandShot LEFT_SHOT = new CandShot(Degrees.of(0), Degrees.of(63), RotationsPerSecond.of(55)); //Temp
        public static final CandShot RIGHT_SHOT = new CandShot(Degrees.of(0), Degrees.of(63), RotationsPerSecond.of(55)); //Temp
        public static final CandShot MIDDLE_SHOT = new CandShot(Degrees.of(0), Degrees.of(80), RotationsPerSecond.of(53)); //Temp
    }

    
    // Subsystems
    private Shooter shooter;
    private Turret turret;
    private Hood hood;
    private Swerve drivetrain;
    private Indexer indexer;

    // Target storage
    private Target storedTarget;

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

        this.storedTarget = FieldConstants.GOAL_POSITION;

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
            storedTarget =  FieldConstants.GOAL_POSITION;
        } else if (isTop) {
            storedTarget =  new Target(FieldConstants.ZONE_POSITION_BLUE_TOP, FieldConstants.ZONE_POSITION_RED_TOP);
        } else {
            storedTarget =  new Target(FieldConstants.ZONE_POSITION_BLUE_BOTTOM, FieldConstants.ZONE_POSITION_RED_BOTTOM);
        }

        updateLogging();
    }

    private void updateLogging() {
        targetPositionLog.append(new double[]{getTargetTranslation().getX(), getTargetTranslation().getY()});
        distToTargetLog.append(getTargetDistance().in(Meters));

        if (Robot.isNTEnabled()) {
            LightningShuffleboard.setTranslation2d("Cannon", "Target Position", FieldConstants.getTargetData(getTarget()));
            // LightningShuffleboard.setTranslation2d("Cannon", "Target Position", getTarget());
            LightningShuffleboard.setPose2d("Cannon", "Target Pose", new Pose2d(getTargetTranslation(), new Rotation2d()));
            LightningShuffleboard.setDouble("Cannon", "Distance To Target", getTargetDistance().in(Meters));
            LightningShuffleboard.setPose2d("Cannon", "Turret Position", new Pose2d(getShooterTranslation(), new Rotation2d()));
        }

        LightningShuffleboard.setBool("Cannon", "In No Passing Zone", isInNoPassingZone());
    }

    /**
     * Gets the translation of the shooter relative to the field
     * @param pose custom pose for the robot
     * @return Returns its position
     */
    public Translation2d getShooterTranslation(Pose2d pose) {
        return pose.transformBy(CannonConstants.SHOOTER_TRANSFORM).getTranslation();
    }

    /**
     * Gets the translation of the shooter relative to the field
     * @return Returns its position
     */
    public Translation2d getShooterTranslation() {
        return getShooterTranslation(drivetrain.getPose());
    }

    /**
     * Gets the translation of the target that we want to aim at (all components)
     * @return The transltion
     */
    public Target getTarget() {
        return storedTarget;
    }

    /**
     * Gets the current target's translati on according to the alliance
     * @return The translation
    */
    public Translation2d getTargetTranslation() {
        return FieldConstants.getTargetData(getTarget());
    }

    public Angle getTargetAngle() {
        return Radians.of(getTargetTranslation().minus(getShooterTranslation()).getAngle().getRadians());
    }

    /**
     * Gets the distance from the robot to the target
     * @return The distance
     */
    public Distance getTargetDistance() {
        return Meters.of(getTargetTranslation().minus(getShooterTranslation()).getNorm());
    }

    /**
     * Gets the distance from the robot to the target
     * @param pose custom pose to pass in
     * @return The distance
     */
    public Distance getTargetDistance(Pose2d pose) {
        return Meters.of(getTargetTranslation().minus(getShooterTranslation(pose)).getNorm());
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
        return new ParallelCommandGroup(
            createCannonCommand(value.hoodAngle, value.shooterVelocity).andThen(hood.idle(), shooter.idle()),
            
            indexWhenOnTarget()
        ).handleInterrupt(() -> {
            shooter.stop();
            turret.stop();
            hood.stop();
            indexer.stop();
        });
        
    }

    /**
     * Creates a cand shot command that also uses turret
     * @param value The cand shot to use
     * @return The command
     */
    public Command createTurretCandShotCommand(CannonConstants.CandShot value) {
      return new ParallelCommandGroup(
            createCannonCommand(value.turretAngle, value.hoodAngle, value.shooterVelocity).andThen(hood.idle(), shooter.idle()),
            indexWhenOnTarget()
        );
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
        return turret.turretAimCommand(this);
    }

    /**
     * Remodel of shooter aim-- automatically decides when to shoot
     * @return The command to run
     */
    public Command smartShoot() {
        return shooter.runShootCommand(() -> Shooter.ShooterConstants.VELOCITY_MAP.get(getTargetDistance()))
        .alongWith(new SequentialCommandGroup(
            new WaitUntilCommand(() -> turret.isOnTarget() && hood.isOnTarget() && shooter.isOnTarget() && !isNearHub()),
            indexer.autoIndex(IndexerConstants.SPINDEXDER_POWER, IndexerConstants.TRANSFER_POWER)
        )
        .finallyDo((end) -> {
            shooter.setPower(ShooterConstants.COAST_DC);
            indexer.stop();
        }));
    }

    /**
     * On-The-Fly shooting command.
     *
     * The convergence loop uses raw doubles (no Measure/Pose2d allocations)
     * to avoid GC pressure on the roboRIO. Measure types are only created
     * at the end when setting motor outputs.
     *
     * @return DA OTFFF
     */
    public Command shootOTF() {
        return new RunCommand(() -> { // hi david (from Bea)
            // ── Snapshot current state (one-time reads) ──────────────

            Pose2d pose = drivetrain.getPose();
            ChassisSpeeds speeds = drivetrain.getCurrentRobotChassisSpeeds();
            Translation2d target = getTargetTranslation();

            double robotX = pose.getX();
            double robotY = pose.getY();
            double sin = pose.getRotation().getSin();
            double cos = pose.getRotation().getCos();

            double targetX = target.getX();
            double targetY = target.getY();

            // Shooter offset in field frame (constant for this loop since dtheta=0)
            double shooterOffX = CannonConstants.SHOOTER_TRANSLATION.getX();
            double shooterOffY = CannonConstants.SHOOTER_TRANSLATION.getY();
            double shooterFieldX = shooterOffX * cos - shooterOffY * sin;
            double shooterFieldY = shooterOffX * sin + shooterOffY * cos;

            // Robot-relative velocity of the shooter offset due to chassis rotation,
            // then rotated to field frame and added to chassis velocity.
            // This matches getFuturePoseFromTime's existing math.
            double rrXVel = -speeds.omegaRadiansPerSecond * shooterOffY;
            double rrYVel =  speeds.omegaRadiansPerSecond * shooterOffX;
            double totalVx = speeds.vxMetersPerSecond + (rrXVel * cos - rrYVel * sin);
            double totalVy = speeds.vyMetersPerSecond + (rrXVel * sin + rrYVel * cos);

            double driveMultiplier = LightningShuffleboard.getDouble("Cannon", "OTF Multiplier", 1);

            // ── Initial state: current shooter position ──────────────

            double futureX = robotX;
            double futureY = robotY;

            double sx = robotX + shooterFieldX;
            double sy = robotY + shooterFieldY;
            double futureDist = Math.sqrt((targetX - sx) * (targetX - sx) + (targetY - sy) * (targetY - sy));

            double toleranceSq = CannonConstants.OTF_TOLERANCE.in(Meters);
            toleranceSq *= toleranceSq; // compare squared to avoid sqrt in convergence check

            // ── Convergence loop (zero allocations) ──────────────────

            for (int i = 0; i < CannonConstants.MAX_OTF_ITERATIONS; i++) {
                double tof = CannonConstants.TIME_OF_FLIGHT_MAP.getBaseUnit(futureDist);
                double dt = tof * driveMultiplier;

                // Predict future robot position: pose.exp(twist) with dtheta=0
                // simplifies to rotating the twist by the heading then translating.
                double twistDx = totalVx * dt;
                double twistDy = totalVy * dt;
                double rawX = robotX + twistDx * cos - twistDy * sin;
                double rawY = robotY + twistDx * sin + twistDy * cos;

                // Under-relaxation: blend toward prediction to guarantee convergence
                double prevX = futureX;
                double prevY = futureY;
                futureX = prevX + CannonConstants.OTF_RELAXATION * (rawX - prevX);
                futureY = prevY + CannonConstants.OTF_RELAXATION * (rawY - prevY);

                // Distance from future shooter position to target
                sx = futureX + shooterFieldX;
                sy = futureY + shooterFieldY;
                futureDist = Math.sqrt((targetX - sx) * (targetX - sx) + (targetY - sy) * (targetY - sy));

                // Convergence check (squared distance avoids sqrt)
                double dx = futureX - prevX;
                double dy = futureY - prevY;
                if (dx * dx + dy * dy < toleranceSq) {
                    break;
                }
            }

            // ── Set motor outputs (allocations only happen here) ─────

            Distance futureDistMeasure = Meters.of(futureDist);
            Angle hoodAngle = Hood.HoodConstants.HOOD_MAP.get(futureDistMeasure);
            AngularVelocity shooterVelocity = Shooter.ShooterConstants.VELOCITY_MAP.get(futureDistMeasure);

            hood.setPosition(hoodAngle);
            shooter.setVelocity(shooterVelocity);

            Pose2d futurePose = new Pose2d(futureX, futureY, pose.getRotation());
            LightningShuffleboard.setPose2d("Cannon", "Future Pose", futurePose);

            turret.turretAim(new Pose2d(getShooterTranslation(futurePose), futurePose.getRotation()), getTarget(), getRobotAngularVelocity(), getHubAngularVelocity());
      }, turret, shooter, hood)
      .alongWith(indexWhenOnTarget().onlyWhile(() -> turret.isOnTarget(Degrees.of(12))).repeatedly());

    //   .alongWith(drivetrain.increaseRampRates())
    //   .alongWith(drivetrain.lowerSupplyLimits());

    }

    /**
     * finds the distance between shooter and hub and calculates if it is nearby.
     * @return if the distance of shooter on the field and the hub is less than 1
     */
    public boolean isNearHub() {
          Distance distance = Meters.of(this.getShooterTranslation().getDistance(FieldConstants.getTargetData(FieldConstants.GOAL_POSITION)));
          return distance.lt(CannonConstants.SMART_SHOOT_MIN_DISTANCE);
    }
    
    /**
     * starts the indexer when the hood, turret, and shooter is on target.
     * @return The command
     */
    public Command indexWhenOnTarget(){
        return new SequentialCommandGroup(
            new WaitUntilCommand(() -> isOnTarget()),
            indexer.autoIndex(IndexerConstants.SPINDEXDER_POWER, Indexer.IndexerConstants.TRANSFER_POWER)
        );
    }

    /**
     * Checks if the cannon is in between the tower and hub on the opposite alliance zone
     * @return if the cannon is in the no passing zone
     */
    public boolean isInNoPassingZone() {
        return AllianceHelpers.isBlueAlliance() ? FieldConstants.RED_NO_PASSING_ZONE.contains(getShooterTranslation()) 
            : FieldConstants.BLUE_NO_PASSING_ZONE.contains(getShooterTranslation());
    }

    /**
     *  Checks if the hood, turret, and shooter is on target.
     * @return if they are on target.
     */
    public boolean isOnTarget(){
        return (hood.isOnTarget() && turret.isOnTarget() && shooter.isOnTarget());
    }

    /**
     * get the angular velocity of the hub based on the robot's velocity and position relative to the hub.
     * @return the angular velocity of the hub based on the robot's velocity and position relative to the hub
     */
    public AngularVelocity getHubAngularVelocity() {
        // double velocityMagnitude = Math.sqrt(Math.pow(drivetrain.getCurrentRobotChassisSpeeds().vxMetersPerSecond, 2) + Math.pow(drivetrain.getCurrentRobotChassisSpeeds().vyMetersPerSecond, 2));
        // double velocityAngle = Math.atan2(drivetrain.getCurrentRobotChassisSpeeds().vyMetersPerSecond, drivetrain.getCurrentRobotChassisSpeeds().vxMetersPerSecond);

        // double[] velocityVector = {velocityMagnitude, velocityAngle};

        // double robotHubMagnitude = getShooterTranslation().getDistance(FieldConstants.getTargetData(FieldConstants.GOAL_POSITION));
        // double robotHubAngle = Math.atan2(FieldConstants.getTargetData(FieldConstants.GOAL_POSITION).getY() - getShooterTranslation().getY(), FieldConstants.getTargetData(FieldConstants.GOAL_POSITION).getX() - getShooterTranslation().getX());

        // double[] robotHubVector = {robotHubMagnitude, robotHubAngle};

        // double[] robotHubOrthogonalVector = {1, robotHubAngle + Math.PI / 2};

        // double orthagVectorScalar = velocityMagnitude * Math.sin(velocityAngle - robotHubAngle);

        // double circumference = 2 * Math.PI * robotHubMagnitude;

        // double angularVelocity = orthagVectorScalar / circumference;
        // return angularVelocity;

        double robotTargetAngle = getTargetAngle().in(Radians);
        
        Translation2d fieldRelativeVelocity = new Translation2d(drivetrain.getCurrentRobotChassisSpeeds().vxMetersPerSecond, drivetrain.getCurrentRobotChassisSpeeds().vyMetersPerSecond).rotateBy(drivetrain.getPose().getRotation());

        double hubRotation = (-fieldRelativeVelocity.getX() * Math.sin(robotTargetAngle) + fieldRelativeVelocity.getY() * Math.cos(robotTargetAngle)) / getTargetDistance().in(Meters);
        return RadiansPerSecond.of(hubRotation);
    }

    public AngularVelocity getRobotAngularVelocity() {
        return RadiansPerSecond.of(drivetrain.getCurrentRobotChassisSpeeds().omegaRadiansPerSecond);
    }
}