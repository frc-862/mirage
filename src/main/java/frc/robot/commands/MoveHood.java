// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;

public class MoveHood extends Command {
  private final Hood hood;
  private Angle hoodAngle;
  private final Supplier<Angle> hoodAngleSupplier;

  /**
   * Basic command for moving the hood
   * @param hood The Hood subsystem
   * @param hoodAngle Target angle for hood
   */

  public MoveHood(Hood hood, Angle hoodAngle) {
    this(hood, () -> hoodAngle);
  }

  public MoveHood(Hood hood, Supplier<Angle> hoodAngleSupplier) {
    this.hood = hood;
    this.hoodAngleSupplier = hoodAngleSupplier;
    addRequirements(hood);
  }

  @Override
  public void initialize() {
    hoodAngle = hoodAngleSupplier.get();
    hood.setPosition(hoodAngle);
  }

  @Override
  public boolean isFinished() {
    return hood.isOnTarget();
  }
}
