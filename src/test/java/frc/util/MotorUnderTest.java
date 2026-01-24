package frc.util;

import java.util.function.DoubleConsumer;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Bundles motor + control hooks + sim objects for testing.
 *
 * @param motor The ThunderBird motor controller
 * @param setPower Function to set motor power
 * @param stop Function to stop the motor
 * @param simState TalonFX simulation state
 * @param sim DC motor physics simulation
 */
public record MotorUnderTest(
    frc.util.hardware.ThunderBird motor,
    DoubleConsumer setPower,
    Runnable stop,
    TalonFXSimState simState,
    DCMotorSim sim
) {
  /**
   * Updates the simulation state by:
   * 1. Setting the supply voltage
   * 2. Getting the motor voltage from sim state
   * 3. Updating the physics simulation
   * 4. Writing position and velocity back to sim state
   *
   * @param dtSeconds Time step for simulation update
   * @param supplyVoltage Battery/supply voltage
   */
  public void updateSim(double dtSeconds, double supplyVoltage) {
    simState.setSupplyVoltage(supplyVoltage);

    var motorVoltage = simState.getMotorVoltage();
    sim.setInputVoltage(motorVoltage);
    sim.update(dtSeconds);

    simState.setRawRotorPosition(sim.getAngularPositionRotations());
    simState.setRotorVelocity(sim.getAngularVelocityRPM() / 60.0); // RPM -> RPS
  }
}
