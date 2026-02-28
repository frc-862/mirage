package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.BaseStatusSignal;
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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
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
import frc.robot.constants.FieldConstants;
import frc.robot.constants.MirageTunerConstants.TunerSwerveDrivetrain;
import frc.util.AllianceHelpers;
import frc.util.simulation.SwerveSim;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class Swerve extends TunerSwerveDrivetrain implements Subsystem {
    private Notifier m_simNotifier = null;
    protected SwerveSim swerveSim;

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
    }

    @Override
    public void simulationPeriodic() {
        resetPose(swerveSim.mapleSimDrive.getSimulatedDriveTrainPose());
    }

    private void startSimThread() {
        /* Increase signal update frequencies to prevent stale CAN frame warnings in sim */
        for (var module : getModules()) {
            BaseStatusSignal.setUpdateFrequencyForAll(1000,
                module.getDriveMotor().getPosition(),
                module.getDriveMotor().getVelocity(),
                module.getSteerMotor().getPosition(),
                module.getSteerMotor().getVelocity()
            );
        }
        BaseStatusSignal.setUpdateFrequencyForAll(1000,
            getPigeon2().getYaw(),
            getPigeon2().getAngularVelocityZWorld()
        );

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
     * Use for autonomous driving ex. auto  align, auton
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
            .withDriveRequestType(DriveRequestType.Velocity)));
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

    public boolean isNearTrench() {
        Translation2d shooterTranslation2d = getPose().getTranslation();
        boolean isNearTrench = FieldConstants.LEFT_BLUE_TRENCH.contains(shooterTranslation2d) ||
            FieldConstants.RIGHT_BLUE_TRENCH.contains(shooterTranslation2d) ||
            FieldConstants.LEFT_RED_TRENCH.contains(shooterTranslation2d) ||
            FieldConstants.RIGHT_RED_TRENCH.contains(shooterTranslation2d);
        return isNearTrench;
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
     * Checks if the robot's pose is within the current alliance's zone
     *
     * @return true if the robot is in the zone, false otherwise
     */
    public boolean isInZone() {
        Pose2d robotPose = getPose();
        if (AllianceHelpers.isRedAlliance()) {
            return (FieldConstants.RED_ALLIANCE_RECT.contains(robotPose.getTranslation()));
        } else {
            return (FieldConstants.BLUE_ALLIANCE_RECT.contains(robotPose.getTranslation()));
        }
    }

    @SuppressWarnings("resource")
    public Command autoAlign(Pose2d targetPose) {
        PIDController pidX = new PIDController(DriveConstants.DRIVE_P, DriveConstants.DRIVE_I, DriveConstants.DRIVE_D);
        PIDController pidY = new PIDController(DriveConstants.DRIVE_P, DriveConstants.DRIVE_I, DriveConstants.DRIVE_D);
        PIDController pidR = new PIDController(DriveConstants.ROT_P, DriveConstants.ROT_I, DriveConstants.ROT_D);

        pidR.enableContinuousInput(-180, 180);

        pidX.setTolerance(DriveConstants.DRIVE_TOLERANCE.in(Meters));
         pidY.setTolerance(DriveConstants.DRIVE_TOLERANCE.in(Meters));
        pidR.setTolerance(DriveConstants.ROT_TOLERANCE.in(Degrees));
        
        return autoDrive(() -> pidX.calculate(getPose().getX(), targetPose.getX()),
        () -> pidY.calculate(getPose().getY(), targetPose.getY()), 
        () -> pidR.calculate(getPose().getRotation().getDegrees(), targetPose.getRotation().getDegrees()));
    }
}
