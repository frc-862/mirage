// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HoodAim extends Command {
  /** Creates a new HoodAim. */
  private Shooter shooter;
  private Hood hood;
  private Translation2d target;
  private Distance distanceToTarget;
  
  /**
   * @param drivetrain
   * @param hood
   * @param targetAngle 
   * @param distanceToTargetMeters
   */
  public HoodAim(Shooter shooter, Hood hood, Translation2d target) {
    this.shooter = shooter;
    this.hood = hood;
    this.target = target;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Translation2d shooterTranslation = shooter.getShooterTranslation();

    //distanceToTarget = Meters.of(robotPose.getTranslation().getDistance(target));
    




    

    };

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //hood.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private Angle OptimizeHoodAngle(Angle desired){
    double targetDegrees = desired.in(Degrees);
    double currentAngle = hood.getAngle().in(Degrees);
    
    double error = targetDegrees - currentAngle;

    
  }
}
