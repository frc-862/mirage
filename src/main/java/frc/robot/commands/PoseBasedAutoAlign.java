// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PoseConstants;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PoseBasedAutoAlign extends Command {
    private Swerve drivetrain;

    private PIDController pidX;
    private PIDController pidY;
    private PIDController pidR;

    private SwerveRequest.FieldCentric AUTO_REQUEST;

    private Pose2d targetPose;

    private final Supplier<Pose2d>targetPoseSupplier;

    /** Creates a new PoseBasedAutoAlign Command.
    * @param drivetrain
    * @param targetPose
    */
    public PoseBasedAutoAlign(Swerve drivetrain, Supplier<Pose2d> targetPoseSupplier) {
        this.drivetrain = drivetrain;

        pidX = new PIDController(PoseConstants.DRIVE_P, PoseConstants.DRIVE_I, PoseConstants.DRIVE_D);
        pidY = new PIDController(PoseConstants.DRIVE_P, PoseConstants.DRIVE_I, PoseConstants.DRIVE_D);
        pidR = new PIDController(PoseConstants.DRIVE_P, PoseConstants.DRIVE_I, PoseConstants.DRIVE_D);

        pidX.setTolerance(PoseConstants.DRIVE_TOLERANCE);
        pidY.setTolerance(PoseConstants.DRIVE_TOLERANCE);
        pidR.setTolerance(PoseConstants.DRIVE_TOLERANCE);

        AUTO_REQUEST = new SwerveRequest.FieldCentric();

        this.targetPoseSupplier = targetPoseSupplier;

        addRequirements(drivetrain);
    }

    public PoseBasedAutoAlign(Swerve drivetrain, Pose2d targetPose) {
        this(drivetrain, () -> targetPose);
    }

    @Override
    public void initialize() {
        targetPose = targetPoseSupplier.get();
    }

    @Override
    public void execute() {
        // uses the autoRequest to set a control for the drivetrain pased on pids
        drivetrain.setControl(autoRequest());
    }

    private SwerveRequest autoRequest(){
        Pose2d currentPose = drivetrain.getPose();

        AUTO_REQUEST.withVelocityX(pidX.calculate(currentPose.getX(), targetPose.getX()));
        AUTO_REQUEST.withVelocityY(pidY.calculate(currentPose.getY(), targetPose.getY()));
        AUTO_REQUEST.withRotationalRate(pidR.calculate(currentPose.getRotation().getDegrees(), targetPose.getRotation().getDegrees()));

        AUTO_REQUEST.withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
        AUTO_REQUEST.withDriveRequestType(DriveRequestType.Velocity);

        return AUTO_REQUEST;
    }

    @Override
    public boolean isFinished() {
        return pidX.atSetpoint() && pidY.atSetpoint() && pidR.atSetpoint();
    }
}
