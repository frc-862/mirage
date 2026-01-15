// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PoseConstants;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PoseBasedAutoAlign extends Command {

  //creates drivertrain variable
  private Swerve drivetrain;

  private PIDController pidX;
  private PIDController pidY;
  private PIDController pidR;

  //Creates a variable for DriveTolerance (amount of possible error)
  private double DriveTolerance = PoseConstants.DRIVE_TOLERANCE;

  //creates a variable for the PID controller for the X-axis (horizontally)

  public Pose2d targetPose = new Pose2d();
  /** Creates a new PoseBasedAutoAlign.
   * @param drivetrain
   * @param targetPose
   */

  public PoseBasedAutoAlign(Swerve drivetrain, Pose2d targetPose) {
    //sets drivetrain
    this.drivetrain = drivetrain;
    this.targetPose = targetPose;

    //sets the pid values to a pid controller
    pidX = new PIDController(PoseConstants.DRIVE_P, PoseConstants.DRIVE_I, PoseConstants.DRIVE_D);
    pidY = new PIDController(PoseConstants.DRIVE_P, PoseConstants.DRIVE_I, PoseConstants.DRIVE_D);
    pidR = new PIDController(PoseConstants.DRIVE_P, PoseConstants.DRIVE_I, PoseConstants.DRIVE_D);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //gives a tolerance to pidX and Y
    pidX.setTolerance(DriveTolerance);
    pidY.setTolerance(DriveTolerance);
    pidR.setTolerance(DriveTolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getPose();

    drivetrain.setControl(new SwerveRequest.FieldCentric()
      .withVelocityX(pidX.calculate(currentPose.getX(), targetPose.getX()))
      .withVelocityY(pidY.calculate(currentPose.getY(), targetPose.getY()))
      .withRotationalRate(pidR.calculate(currentPose.getRotation().getDegrees(), targetPose.getRotation().getDegrees())));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
