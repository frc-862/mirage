// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Spindexer;

public class Spin extends Command {
  private Spindexer spindexer;
  private double power;

  /**
   * Moves the spindexer to transfer fuel to cannon
   *
   * @param spindexer The Spindexer subsystem
   * @param power Motor power
   */
  
  public Spin(Spindexer spindexer, double power) {
    this.spindexer = spindexer;
    addRequirements(spindexer);
  }

  @Override
  public void initialize() {
    spindexer.setPower(power);
  }

  @Override
  public void end(boolean interrupted) {
    spindexer.stop();
  }
}
