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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.RobotMap;
import frc.util.hardware.ThunderBird;
import frc.util.shuffleboard.LightningShuffleboard;

public class Indexer extends SubsystemBase {
    private final ThunderBird spindexerMotor;
    private final ThunderBird transferMotor;

    private final DutyCycleOut spindexerDutyCycle;
    private final DutyCycleOut transferDutyCycle;

    private LinearSystemSim<N1, N1, N1> spindexerSim;
    private TalonFXSimState motorSim;

    /** Creates a new Spindexer Subsystem. */
    public Indexer() {
        spindexerMotor = new ThunderBird(RobotMap.SPINDEXER, RobotMap.CAN_BUS,
            IndexerConstants.SPINDEXER_MOTOR_INVERTED, IndexerConstants.SPINDEXER_MOTOR_STATOR_LIMIT, IndexerConstants.SPINDEXER_MOTOR_BRAKE_MODE);

        transferMotor = new ThunderBird(RobotMap.TRANSFER, RobotMap.CAN_BUS,
            IndexerConstants.TRANSFER_MOTOR_INVERTED, IndexerConstants.TRANSFER_MOTOR_STATOR_LIMIT, IndexerConstants.TRANSFER_MOTOR_BRAKE_MODE);

        spindexerDutyCycle = new DutyCycleOut(0d);
        transferDutyCycle = new DutyCycleOut(0d);

        if (Robot.isSimulation()) {
            spindexerSim = new LinearSystemSim<N1, N1, N1>(LinearSystemId.identifyVelocitySystem(IndexerConstants.SPINDEXER_SIM_kV,
                IndexerConstants.SPINDEXER_SIM_kA));
            motorSim = spindexerMotor.getSimState();
            motorSim.setMotorType(MotorType.KrakenX44);
        }
    }

    @Override
    public void simulationPeriodic() {
        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        spindexerSim.setInput(motorSim.getMotorVoltageMeasure().in(Volts));
        spindexerSim.update(Robot.kDefaultPeriod);

        motorSim.setRotorVelocity(spindexerSim.getOutput(0));

        LightningShuffleboard.setDouble("Spindexer", "Velocity", getSpindexerVelocity().in(RotationsPerSecond));
    }

    /**
     * Set the power of the spindexer motor using duty cycle out
     * @param power duty cycle value from -1.0 to 1.0
     */
    public void setSpindexerPower(double power) {
        spindexerMotor.setControl(spindexerDutyCycle.withOutput(power));
    }

    /**
     * Gets the current velocity of the spindexer motor.
     * @return the spindexer motor velocity as an {@link AngularVelocity}
     */
    public AngularVelocity getSpindexerVelocity() {
        return spindexerMotor.getVelocity().getValue();
    }

    /**
     * stops all movement to the spindexer motor
     */
    public void stopSpindexer() {
        spindexerMotor.stopMotor();
    }

    /**
     * Set the power of the transfer motor using duty cycle out
     * @param power duty cycle value from -1.0 to 1.0
     */
    public void setTransferPower(double power) {
        transferMotor.setControl(transferDutyCycle.withOutput(power));
    }

    /**
     * stops all movement to the transfer motor
     */
    public void stopTransfer() {
        transferMotor.stopMotor();
    }

    /**
     * Set the power to the overall indexer using duty cycle out
     * @param power duty cycle value from -1.0 to 1.0
     */
    public void setPower(double power) {
        setSpindexerPower(power);
        setTransferPower(power);
    }

    /**
     * stops all movement to the overall indexer
     */
    public void stop() {
        stopSpindexer();
        stopTransfer();
    }

    public Command Index(double power) {
        Command index = new Command() {

            @Override
            public void initialize() {
                setSpindexerPower(power);
            }

            @Override
            public void end(boolean interrupted) {
                stop();
            }
        };

        return index;
    }
}
