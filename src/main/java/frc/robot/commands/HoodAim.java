// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;

public class HoodAim extends Command {
  private Hood hood;
  private double power;

  /**
   * Moves the hood using target power
   *
   * @param hood The Hood subsystem
   * @param power Motor power
   */

  public HoodAim(Hood hood, double power) {
    this.hood = hood;
    addRequirements(hood);
  }

  @Override
  public void initialize() {
    hood.setPower(power);
  }

  @Override
  public void end(boolean interrupted) {
    hood.stop();
  }
}
