// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;

public class MoveHood extends Command {
  private final Hood hood;
  private final Angle hoodAngle;

  /**
   * Basic command for moving the hood
   * @param hood The Hood subsystem
   * @param hoodAngle Target angle for hood
   */

  public MoveHood(Hood hood, Angle hoodAngle) {
    this.hood = hood;
    this.hoodAngle = hoodAngle;
    addRequirements(hood);
  }

  @Override
  public void initialize() {
    hood.setPosition(hoodAngle);
  }

  @Override
  public void end(boolean interrupted) {
    hood.stop();
  }

  @Override
  public boolean isFinished() {
    return hood.isOnTarget();
  }
}
