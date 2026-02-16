package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.MirageTunerConstants.TunerSwerveDrivetrain;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.util.AllianceHelpers;
import frc.util.shuffleboard.LightningShuffleboard;
import frc.util.simulation.SwerveSim;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class Swerve extends TunerSwerveDrivetrain implements Subsystem {
    private Notifier m_simNotifier = null;
    protected SwerveSim swerveSim;
    private Translation2d targetPosition = new Translation2d(0, 0);

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    public static class FieldConstants {
        private record Target(Translation2d blue, Translation2d red) {}

        public static final Target GOAL_POSITION = new Target(new Translation2d(4.625594, 4.034536), new Translation2d(11.915394, 4.034536));
        public static final Target DEPOT_POSITION = new Target(new Translation2d(0.3937, 0.665988), new Translation2d(16.147288, 7.403338));

        public static Translation2d getTargetData(Target target) {
            return AllianceHelpers.isBlueAlliance() ? target.blue() : target.red();
        }

        // All Rectangle2ds probably have to be changed
        public static final Rectangle2d BLUE_ALLIANCE_RECT = new Rectangle2d(new Pose2d(2.312797, 4.034663, new Rotation2d()), 4.625594, 8.069326); // temp
        public static final Rectangle2d RED_ALLIANCE_RECT = new Rectangle2d(new Pose2d(14.228191, 4.034663, new Rotation2d()), 4.625594, 8.069326); // temp
        public static final Rectangle2d BOTTOM_HALF_RECT = new Rectangle2d(new Pose2d(8.270494, 2.017268, new Rotation2d()), 16.540988, 4.034663); // temp - side on left from perspective of blue driverstation
        public static final Rectangle2d TOP_HALF_RECT = new Rectangle2d(new Pose2d(8.270494, 6.052291, new Rotation2d()), 16.540988, 4.034663); // temp - side on right from perspective of blue driverstation

        public static final Translation2d ZONE_POSITION_BLUE_TOP = new Translation2d(2.034536, 5.963158);
        public static final Translation2d ZONE_POSITION_BLUE_BOTTOM = new Translation2d(2.034536, 2.105914);

        public static final Translation2d ZONE_POSITION_RED_TOP = new Translation2d(13.915394, 5.963158);
        public static final Translation2d ZONE_POSITION_RED_BOTTOM = new Translation2d(13.915394, 2.105914);

        public static final double FIELD_MIDDLE_Y = 4.034663;
    }
    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineSteer;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public Swerve(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, SwerveSim.regulateModuleConstantsForSimulation(modules));

        configurePathplanner();

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public Swerve(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, SwerveSim.regulateModuleConstantsForSimulation(modules));

        configurePathplanner();

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public Swerve(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
            SwerveSim.regulateModuleConstantsForSimulation(modules));

        configurePathplanner();

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param requestSupplier Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
        
        targetPosition = findTargetPosition();
    }

    @Override
    public void simulationPeriodic() {
        resetPose(swerveSim.mapleSimDrive.getSimulatedDriveTrainPose());
    }

    private void startSimThread() {
        swerveSim = DriveConstants.getSwerveSim(this);
        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(swerveSim::update);
        m_simNotifier.startPeriodic(DriveConstants.kSimLoopPeriod.in(Seconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    // /**
    //  * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
    //  * while still accounting for measurement noise.
    //  * <p>
    //  * Note that the vision measurement standard deviations passed into this method
    //  * will continue to apply to future measurements until a subsequent call to
    //  * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
    //  *
    //  * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
    //  * @param timestampSeconds The timestamp of the vision measurement in seconds.
    //  * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
    //  *     in the form [x, y, theta]ᵀ, with units in meters and radians.
    //  */
    // @Override
    // public void addVisionMeasurement(
    //     Pose2d visionRobotPoseMeters,
    //     double timestampSeconds,
    //     Matrix<N3, N1> visionMeasurementStdDevs
    // ) {
    //     super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    // }

    public void addVisionMeasurement(EstimatedRobotPose pose, double distance) {
        if (DriverStation.isDisabled()) {
            addVisionMeasurement(pose.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(pose.timestampSeconds),
                    VecBuilder.fill(0.1, 0.1, 0.1));
        } else {
            // addVisionMeasurement(pose.estimatedPose.toPose2d(),
            // Utils.fpgaToCurrentTime(pose.timestampSeconds),
            // VecBuilder.fill(VisionConstants.VISION_X_STDEV,
            // VisionConstants.VISION_Y_STDEV, VisionConstants.VISION_THETA_STDEV));

            // if(distance < 0.25) {
            // addVisionMeasurement(pose.estimatedPose.toPose2d(),
            // Utils.fpgaToCurrentTime(pose.timestampSeconds),
            // VecBuilder.fill(0.01, 0.01, 0.01));
            // } else {

            // for ambiguity-based (or distance-based) std deviations
            addVisionMeasurement(pose.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(pose.timestampSeconds),
                    VecBuilder.fill(distance / 2, distance / 2, distance / 2));
            // }

        }
    }

    /**
     * Use for autonomous driving ex. auto align, auton
     * Use whenever you want a specific velocity with closed loop control
     * Always uses blue alliace perspective
     * @param xInput input for the x velocity
     * @param yInput input for the x velocity
     * @param rInput input for the x velocity
     * @return command to run
     */
    public Command autoDrive(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier rInput) {
        return run(() -> setControl(DriveConstants.fieldCentricRequest
            .withVelocityX(DriveConstants.MaxSpeed.times(xInput.getAsDouble()))
            .withVelocityY(DriveConstants.MaxSpeed.times(yInput.getAsDouble()))
            .withRotationalRate(DriveConstants.MaxAngularRate.times(rInput.getAsDouble()))
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)));
    }

    public Command driveCommand(Supplier<Vector<N2>> xyInput, DoubleSupplier rInput) {
        return run(() -> setControl(DriveConstants.fieldCentricRequest
            .withVelocityX(DriveConstants.MaxSpeed.times(xyInput.get().get(0)))
            .withVelocityY(DriveConstants.MaxSpeed.times(xyInput.get().get(1)))
            .withRotationalRate(DriveConstants.MaxAngularRate.times(rInput.getAsDouble()))
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)));
    }

    public Command robotCentricDrive(Supplier<Vector<N2>> xyInput, DoubleSupplier rInput){
        return run(() -> setControl(DriveConstants.robotCentricRequest
            .withVelocityX(DriveConstants.MaxSpeed.times(xyInput.get().get(0)))
            .withVelocityY(DriveConstants.MaxSpeed.times(xyInput.get().get(1)))
            .withRotationalRate(DriveConstants.MaxAngularRate.times(rInput.getAsDouble()))
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)));
    }

    public Command brakeCommand() {
        return applyRequest(() -> DriveConstants.brakeRequest);
    }

    public Command resetFieldCentricCommand() {
        return new InstantCommand(() -> seedFieldCentric());
    }

    public Pose2d getPose(){
        return getState().Pose;
    }

    /**
     * Getting the position of the shooter on the field.
     * @return The translation of the shooter.
     */
    public Translation2d getShooterTranslation() {
        return getPose().getTranslation().plus(ShooterConstants.SHOOTER_POSITION_ON_ROBOT.rotateBy(getPose().getRotation()));
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds(){
        return getState().Speeds;
    }

    public void configurePathplanner(){
        AutoBuilder.configure(
            this::getPose, // Supplier of current robot pose
            this::resetPose, // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
                (speeds, feedforwards) -> this
                    .setControl(DriveConstants.autonRequest.withSpeeds(speeds)
                        .withDriveRequestType(DriveRequestType.Velocity)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY())), // Consumer of
                                                                                                    // ChassisSpeeds to
                                                                                                    // drive the robot
            new PPHolonomicDriveController(DriveConstants.TRANSLATION_PID, DriveConstants.ROTATION_PID),
            DriveConstants.getConfig(getModuleLocations()),
            () -> AllianceHelpers.isRedAlliance(),
            this); // Subsystem for requirements
    }

    /**
     * Finds the optimal target position on the field based on the robot's pose
     *
     * @return Translation2d of the last-computed target position (updated in periodic())
     */
    public Translation2d findTargetPosition() {
        Pose2d robotPose = getPose();

        if (isInZone()) {
            return FieldConstants.getTargetData(FieldConstants.GOAL_POSITION);
        }
        
        boolean isTop = robotPose.getY() > FieldConstants.FIELD_MIDDLE_Y;

        if (AllianceHelpers.isBlueAlliance()) {
            return isTop ? FieldConstants.ZONE_POSITION_BLUE_TOP : FieldConstants.ZONE_POSITION_BLUE_BOTTOM;
        } else {
            return isTop ? FieldConstants.ZONE_POSITION_RED_TOP : FieldConstants.ZONE_POSITION_RED_BOTTOM;
        }
    }

    public Translation2d getTargetPosition() {
        return targetPosition;
    }

    /**
     * Checks if the robot's pose is within the current alliance's zone
     *
     * @return true if the robot is in the zone, false otherwise
     */
    public boolean isInZone() {
        Pose2d robotPose = getPose();
        if (AllianceHelpers.isRedAlliance()) {
            return (Swerve.FieldConstants.RED_ALLIANCE_RECT.contains(robotPose.getTranslation()));
        } else {
            return (Swerve.FieldConstants.BLUE_ALLIANCE_RECT.contains(robotPose.getTranslation()));
        }
    }
}
