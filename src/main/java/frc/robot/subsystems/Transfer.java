// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.RobotMap;
import frc.robot.constants.TurretConstants;
import frc.util.hardware.ThunderBird;
import frc.util.shuffleboard.LightningShuffleboard;

public class Transfer extends SubsystemBase {
  ThunderBird transfer;

  private LinearSystemSim<N1, N1, N1> transferSim;
  private TalonFXSimState motorSim;

  /** Creates a new Transfer. */
  public Transfer() {
    transfer = new ThunderBird(RobotMap.TRANSFER_MOTOR_ID,
    RobotMap.CAN_BUS, TurretConstants.TRANSFER_MOTOR_INVERTED,
    TurretConstants.TRANSFER_MOTOR_STATOR_LIMIT,
    TurretConstants.TRANSFER_MOTOR_BRAKE_MODE);

    if (Robot.isSimulation()) {
      transferSim = new LinearSystemSim<N1, N1, N1>(LinearSystemId.identifyVelocitySystem(TurretConstants.SIM_kV,
          TurretConstants.SIM_kA));
      motorSim = transfer.getSimState();
      motorSim.setMotorType(MotorType.KrakenX44);
    }
  }

  @Override
  public void simulationPeriodic() {
    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    transferSim.setInput(motorSim.getMotorVoltageMeasure().in(Volts));
    transferSim.update(Robot.kDefaultPeriod);

    motorSim.setRotorVelocity(transferSim.getOutput(0));

    LightningShuffleboard.setDouble("Spindexer","Velocity", getVelocity().in(RotationsPerSecond));
  }

  /* Turns the motor for the transfer on */
  public void setPower(double power) {
    transfer.setControl(new DutyCycleOut(power));
    }

    public AngularVelocity getVelocity() {
        return transfer.getVelocity().getValue();
    }
  /* Stops the transfer motor  */
  public void stop() {
    transfer.stopMotor();
    }
}