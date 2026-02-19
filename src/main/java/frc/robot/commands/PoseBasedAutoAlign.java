// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.Swerve;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PoseBasedAutoAlign extends Command {

    public class PoseConstants {
        public static final double DRIVE_P = 1.5d;
        public static final double DRIVE_I = 0;
        public static final double DRIVE_D = 0.08;
        public static final Distance DRIVE_TOLERANCE = Meters.of(0.05);
        public static final double DRIVE_KS = 0;

        public static final double ROT_P = 0.03;
        public static final double ROT_I = 0;
        public static final double ROT_D = 0;
        public static final Angle ROT_TOLERANCE = Degrees.of(1.5);
        public static final double ROT_KS = 0; // 0.01 NOT APPLIED
    }

    private Swerve drivetrain;

    private PIDController pidX;
    private PIDController pidY;
    private PIDController pidR;

    private Pose2d targetPose;

    private final Supplier<Pose2d> targetPoseSupplier;

    /** Creates a new PoseBasedAutoAlign Command.
    * @param drivetrain
    * @param targetPoseSupplier
    */
    public PoseBasedAutoAlign(Swerve drivetrain, Supplier<Pose2d> targetPoseSupplier) {

        this.drivetrain = drivetrain;

        pidX = new PIDController(PoseConstants.DRIVE_P, PoseConstants.DRIVE_I, PoseConstants.DRIVE_D);
        pidY = new PIDController(PoseConstants.DRIVE_P, PoseConstants.DRIVE_I, PoseConstants.DRIVE_D);
        pidR = new PIDController(PoseConstants.ROT_P, PoseConstants.ROT_I, PoseConstants.ROT_D);

        pidX.setTolerance(PoseConstants.DRIVE_TOLERANCE.in(Meters));
        pidY.setTolerance(PoseConstants.DRIVE_TOLERANCE.in(Meters));
        pidR.setTolerance(PoseConstants.ROT_TOLERANCE.in(Degrees));

        pidR.enableContinuousInput(-180, 180);

        this.targetPoseSupplier = targetPoseSupplier;

        addRequirements(drivetrain);
    }

    public PoseBasedAutoAlign(Swerve drivetrain, Pose2d targetPose) {
        this(drivetrain, () -> targetPose);
    }

    @Override
    public void initialize() {
        pidX.reset();
        pidY.reset();
        pidR.reset();
        
        targetPose = targetPoseSupplier.get();
    }

    @Override
    public void execute() {
        drivetrain.setControl(getRequest());
    }

    private SwerveRequest getRequest(){
        Pose2d currentPose = drivetrain.getPose();

        return DriveConstants.fieldCentricRequest.withVelocityX(pidX.calculate(currentPose.getX(), targetPose.getX()))
            .withVelocityY(pidY.calculate(currentPose.getY(), targetPose.getY()))
            .withRotationalRate(pidR.calculate(currentPose.getRotation().getDegrees(), targetPose.getRotation().getDegrees()))
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withDriveRequestType(DriveRequestType.Velocity);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(DriveConstants.brakeRequest);
    }

    @Override
    public boolean isFinished() {
        return pidX.atSetpoint() && pidY.atSetpoint() && pidR.atSetpoint();
    }
}
