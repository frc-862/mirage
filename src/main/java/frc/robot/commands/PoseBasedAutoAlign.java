// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.PoseConstants;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PoseBasedAutoAlign extends Command {
    // creates drivetrain variable
    private Swerve drivetrain;

    // create variables for all the pids as PIDControllers
    private PIDController pidX;
    private PIDController pidY;
    private PIDController pidR;

    private Pose2d targetPose;

    /** Creates a new PoseBasedAutoAlign Command.
    * @param drivetrain
    * @param targetPose
    */
    public PoseBasedAutoAlign(Swerve drivetrain, Pose2d targetPose) {
        // sets drivetrain
        this.drivetrain = drivetrain;

        // sets the pid values to a pid controller
        pidX = new PIDController(PoseConstants.DRIVE_P, PoseConstants.DRIVE_I, PoseConstants.DRIVE_D);
        pidY = new PIDController(PoseConstants.DRIVE_P, PoseConstants.DRIVE_I, PoseConstants.DRIVE_D);
        pidR = new PIDController(PoseConstants.DRIVE_P, PoseConstants.DRIVE_I, PoseConstants.DRIVE_D);

        pidX.setTolerance(PoseConstants.DRIVE_TOLERANCE.in(Meters));
        pidY.setTolerance(PoseConstants.DRIVE_TOLERANCE.in(Meters));
        pidR.setTolerance(PoseConstants.ROT_TOLERANCE.in(Degrees));

        // sets target pose
        this.targetPose = targetPose;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // uses the autoRequest to set a control for the drivetrain pased on pids
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
    public boolean isFinished() {
        return pidX.atSetpoint() && pidY.atSetpoint() && pidR.atSetpoint();
    }
}
