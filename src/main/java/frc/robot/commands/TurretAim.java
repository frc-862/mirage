// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurretAim extends Command {
  Swerve drivetrain;
  Translation2d target;
  Turret turret;

  double targetAngle;

  /** Creates a new TurretAim. */
  public TurretAim(Swerve drivetrain, Turret turret, Translation2d target) {
    this.drivetrain = drivetrain;
    this.target = target;
    this.turret = turret;

    targetAngle = 0;

    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get our robot pose
    Pose2d robotPose = drivetrain.getPose();

    // Get the translation assuming the robot at (0, 0)
    Translation2d delta = target.minus(robotPose.getTranslation());

    // Well add values if we have to based on however the angles acually get calculated
    targetAngle = Math.toDegrees(Math.atan2(delta.getY(), delta.getX()));

    // Set the turret angle based off our robot angle as well
    turret.setAngle(robotPose.getRotation().getDegrees() - targetAngle);    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if Math.abs(turretTargetAngle - turret.getAngle()) > someTolerance
    return Math.abs(targetAngle - turret.getAngle()) < TurretConstants.TURRET_ANGLE_TOLERANCE;

  }
}
