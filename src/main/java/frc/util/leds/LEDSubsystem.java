// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.util.leds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  private final LEDController leds;

  private final boolean[] enabledStates;
  private final LEDBehavior[] ledBehaviors;
  private LEDBehavior defaultBehavior;

  /** 
   * Creates a new nLEDs. </p>
   * nLEDs is a subsystem for managing complex sets of LED behaviors. 
   * 
   * @param numStates The number of different LED states to manage.
   * @param numLEDs The number of LEDs in the LED controller.
   * @param pwmPort The PWM port the LED controller is connected to.
  */
  public LEDSubsystem(int numStates, int numLEDs, int pwmPort) {
    enabledStates = new boolean[numStates];
    ledBehaviors = new LEDBehavior[numStates];

    leds = new LEDController(numLEDs, pwmPort);

    defaultBehavior = new LEDBehavior() { @Override public void apply(LEDController leds) {} };
  }

  /**
   * Sets the LED behavior for a specific state ID.
   * 
   * @param stateID The ID of the state to set the behavior for.
   * @param behavior The LED behavior to set for the state.
   */
  public void setBehavior(int stateID, LEDBehavior behavior) {
    ledBehaviors[stateID] = behavior;
  }

  /**
   * Sets the default LED behavior.
   * 
   * @param behavior The default LED behavior to set.
   */
  public void setDefaultBehavior(LEDBehavior behavior) {
    defaultBehavior = behavior;
  }

  /**
   * Creates a command that enables a specific state. </p>
   * This command will enable the state when started and disable it when ended.
   * 
   * @param stateID The ID of the state to set.
   * @return A command that enables the specified state.
   */
  public Command enableState(int stateID) {
    return new StartEndCommand(
      () -> { enabledStates[stateID] = true; }, 
      () -> { enabledStates[stateID] = false; })
      .ignoringDisable(true);
  }

  /**
   * Creates a command that enables a specific state. </p>
   * This command will enable the state when started and disable it when ended.
   * 
   * @param stateID The ID of the state to set.
   * @param timeoutSeconds The length of time to enable the state for.
   * @return A command that enables the specified state.
   */
  public Command enableStateWithTimeout(int stateID, double timeoutSeconds) {
    return enableState(stateID).withTimeout(timeoutSeconds);
  }

  /**
   * Gets if a specific state is enabled.
   * 
   * @param stateID The ID of the state to get.
   * @return true if the state is enabled, false otherwise.
   */
  public boolean getState(int stateID) {
    return enabledStates[stateID];
  }
  
  @Override
  public void periodic() {
    defaultBehavior.apply(leds);
    for (int i = enabledStates.length - 1; i >= 0; i--) {
      LEDBehavior behavior = ledBehaviors[i];
      if (enabledStates[i] && behavior != null) {
        behavior.apply(leds);
      }
    }

    leds.apply();
  }

}
