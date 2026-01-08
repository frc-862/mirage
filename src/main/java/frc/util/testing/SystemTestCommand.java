package frc.util.testing;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public class SystemTestCommand extends Command {

  public SystemTestCommand() {}

  protected boolean canRun() {
    return DriverStation.isTest();
  }

  protected void initializeTest() {}

  protected void executeTest() {}

  protected void endTest(boolean interrupted) {}

  @Override
  public void initialize() {
    if (canRun()) {
      initializeTest();
    }
  }

  @Override
  public void execute() {
    if (canRun()) {
      executeTest();
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (canRun()) {
      endTest(interrupted);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}