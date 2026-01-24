// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Transfer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BasicTransfer extends Command {
  private Transfer transfer;
  private double speed;

  /**
   * Transfers fuel from spindexer to cannon
   *
   * @param transfer The Transfer subsystem
   * @param speed Motor's speed
   */

  public BasicTransfer(Transfer transfer, double speed) {
    this.transfer = transfer;

    addRequirements(transfer);
  }

  @Override
  public void initialize() {
    transfer.setPower(speed);
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {
    transfer.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
