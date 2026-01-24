// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Spindexer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Spin extends Command {
  /** Creates a new BasicSpindex. */
  private Spindexer spindexer;
  private double power;

  public Spin(Spindexer spindexer, double power) {
    this.spindexer = spindexer;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(spindexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    spindexer.setPower(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    spindexer.stop();
  }
}
