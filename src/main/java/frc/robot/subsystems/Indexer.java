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
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.RobotMap;
import frc.util.hardware.ThunderBird;
import frc.util.shuffleboard.LightningShuffleboard;

public class Indexer extends SubsystemBase {
    private final ThunderBird indexerMotor;
    private final DutyCycleOut dutyCycle;

    private LinearSystemSim<N1, N1, N1> indexerSim;
    private TalonFXSimState motorSim;

    /** Creates a new Indexer. */
    public Indexer() {
        indexerMotor = new ThunderBird(RobotMap.SPINDEXER_MOTOR_ID, RobotMap.CAN_BUS,
            IndexerConstants.INDEXER_MOTOR_INVERTED, IndexerConstants.INDEXER_MOTOR_STATOR_LIMIT, IndexerConstants.INDEXER_MOTOR_BRAKE_MODE);

        dutyCycle = new DutyCycleOut(0);

        if (Robot.isSimulation()) {
            indexerSim = new LinearSystemSim<N1, N1, N1>(LinearSystemId.identifyVelocitySystem(IndexerConstants.SIM_kV,
                IndexerConstants.SIM_kA));
            motorSim = indexerMotor.getSimState();
            motorSim.setMotorType(MotorType.KrakenX44);
        }
    }

    @Override
    public void simulationPeriodic() {
        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        indexerSim.setInput(motorSim.getMotorVoltageMeasure().in(Volts));
        indexerSim.update(Robot.kDefaultPeriod);

        motorSim.setRotorVelocity(indexerSim.getOutput(0));

        LightningShuffleboard.setDouble("Indexer", "Velocity", getVelocity().in(RotationsPerSecond));
    }

    /**
     * Sets power to (-1 to 1) to the indexer motor.
     * @param power
     */
    public void setPower(double power) {
        indexerMotor.setControl(dutyCycle.withOutput(power));
    }

    /**
     * Gets the current velocity of the indexer motor.
     *
     * @return the indexer motor velocity as an {@link AngularVelocity}
     */
    public AngularVelocity getVelocity() {
        return indexerMotor.getVelocity().getValue();
    }

    /**
     * Stops the indexer motor.
     */
    public void stop() {
        indexerMotor.stopMotor();
    }
}
