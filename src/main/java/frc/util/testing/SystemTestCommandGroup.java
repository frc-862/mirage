// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.util.testing;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SystemTestCommandGroup extends SystemTestCommand {

  protected SequentialCommandGroup commandGroup;

  public SystemTestCommandGroup(SequentialCommandGroup commandGroup) {
    this.commandGroup = commandGroup;
  }

  @Override
  protected void initializeTest() {
    commandGroup.schedule();
  }

  @Override
  protected void executeTest() {}

  @Override
  protected void endTest(boolean interrupted) {
    commandGroup.cancel();
  }
}
