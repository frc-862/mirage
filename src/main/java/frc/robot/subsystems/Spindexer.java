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
import frc.robot.constants.SpindexerConstants;
import frc.robot.constants.RobotMap;
import frc.util.hardware.ThunderBird;
import frc.util.shuffleboard.LightningShuffleboard;

public class Spindexer extends SubsystemBase {
    private final ThunderBird spindexerMotor;
    private final DutyCycleOut dutyCycle;

    private LinearSystemSim<N1, N1, N1> spindexerSim;
    private TalonFXSimState motorSim;

    /** Creates a new Spindexer Subsystem. */
    public Spindexer() {
        spindexerMotor = new ThunderBird(RobotMap.SPINDEXER_MOTOR_ID, RobotMap.CAN_BUS,
            SpindexerConstants.INVERTED, SpindexerConstants.STATOR_LIMIT, SpindexerConstants.BRAKE_MODE);

        dutyCycle = new DutyCycleOut(0);

        if (Robot.isSimulation()) {
            spindexerSim = new LinearSystemSim<N1, N1, N1>(LinearSystemId.identifyVelocitySystem(SpindexerConstants.SIM_kV,
                SpindexerConstants.SIM_kA));
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

        LightningShuffleboard.setDouble("Spindexer", "Velocity", getVelocity().in(RotationsPerSecond));
    }

    /**
     * Sets power to (-1 to 1) to the spindexer motor.
     * @param power
     */
    public void setPower(double power) {
        spindexerMotor.setControl(dutyCycle.withOutput(power));
    }

    /**
     * Gets the current velocity of the spindexer motor.
     *
     * @return the spindexer motor velocity as an {@link AngularVelocity}
     */
    public AngularVelocity getVelocity() {
        return spindexerMotor.getVelocity().getValue();
    }

    /**
     * Stops the spindexer motor.
     */
    public void stop() {
        spindexerMotor.stopMotor();
    }
}
