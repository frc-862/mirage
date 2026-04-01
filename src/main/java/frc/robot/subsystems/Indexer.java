// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.plant.LinearSystemId;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.constants.RobotMap;
import frc.util.hardware.ThunderBird;
import frc.util.shuffleboard.LightningShuffleboard;

public class Indexer extends SubsystemBase {

    public class IndexerConstants {
        // spindexer
        public static final boolean SPINDEXER_MOTOR_INVERTED = true; // temp
        public static final Current SPINDEXER_MOTOR_STATOR_LIMIT = Amps.of(40); // temp
        public static final boolean SPINDEXER_MOTOR_BRAKE_MODE = true; // temp

        public static final Current SPINDEXER_SUPPLY_LIMIT = Amps.of(40); // temp
        public static final boolean SPINDEXER_SUPPLY_LIMIT_ENABLE = true; // temp

        public static final double SPINDEXER_SIM_kV = 0.24;
        public static final double SPINDEXER_SIM_kA = 0.8;
        public static final double SPINDEXDER_POWER = 1;

        public static final Time SPINDEXER_DELAY = Seconds.of(0.25);

        // transfer
        public static final boolean TRANSFER_MOTOR_INVERTED = true; // temp
        public static final Current TRANSFER_MOTOR_STATOR_LIMIT = Amps.of(40); // temp
        public static final boolean TRANSFER_MOTOR_BRAKE_MODE = true; // temp

        public static final double TRANSFER_POWER = 0.7;
        public static final double TRANSFER_VELOCITY = 70;
        public static final Current TRANSFER_SUPPLY_LIMIT = Amps.of(40); // temp
        public static final boolean TRANSFER_SUPPLY_LIMIT_ENABLE = true; // temp

        public static final double TRANSFER_kP = 0.09; // temp
        public static final double TRANSFER_kV_FEEDFORWARD = 0.092; // temp

        // sim
        public static final AngularVelocity SIM_INDEX_THRESHOLD = RotationsPerSecond.of(1); // temp
    }

    private final ThunderBird spindexerMotor;
    private final ThunderBird transferMotor;

    private final DutyCycleOut spindexerDutyCycle;
    private final DutyCycleOut transferDutyCycle;

    private VelocityVoltage transferVelocityVoltage;

    private LinearSystemSim<N1, N1, N1> spindexerSim;
    private TalonFXSimState motorSim;

    private DoubleLogEntry transferLog;
    private DoubleLogEntry spindexerLog; 

    /** Creates a new Spindexer Subsystem. */
    public Indexer() {
        spindexerMotor = new ThunderBird(RobotMap.SPINDEXER, RobotMap.CAN_BUS,
            IndexerConstants.SPINDEXER_MOTOR_INVERTED, IndexerConstants.SPINDEXER_MOTOR_STATOR_LIMIT, IndexerConstants.SPINDEXER_MOTOR_BRAKE_MODE);

        transferMotor = new ThunderBird(RobotMap.TRANSFER, RobotMap.CAN_BUS,
            IndexerConstants.TRANSFER_MOTOR_INVERTED, IndexerConstants.TRANSFER_MOTOR_STATOR_LIMIT, IndexerConstants.TRANSFER_MOTOR_BRAKE_MODE);

        spindexerDutyCycle = new DutyCycleOut(0d);
        transferDutyCycle = new DutyCycleOut(0d);
        VelocityVoltage transferVelocityVoltage = new VelocityVoltage(0);

        TalonFXConfiguration spindexerConfig = spindexerMotor.getConfig();
        TalonFXConfiguration transferConfig = transferMotor.getConfig();

        spindexerConfig.CurrentLimits.SupplyCurrentLimitEnable = IndexerConstants.SPINDEXER_SUPPLY_LIMIT_ENABLE;
        spindexerConfig.CurrentLimits.SupplyCurrentLimit = IndexerConstants.SPINDEXER_SUPPLY_LIMIT.in(Amps);

        transferConfig.Slot0.kP = IndexerConstants.TRANSFER_kP;

        transferConfig.CurrentLimits.SupplyCurrentLimitEnable = IndexerConstants.TRANSFER_SUPPLY_LIMIT_ENABLE;
        transferConfig.CurrentLimits.SupplyCurrentLimit = IndexerConstants.TRANSFER_SUPPLY_LIMIT.in(Amps);

        spindexerMotor.applyConfig(spindexerConfig);
        transferMotor.applyConfig(transferConfig);

        if (Robot.isSimulation()) {
            spindexerSim = new LinearSystemSim<N1, N1, N1>(LinearSystemId.identifyVelocitySystem(IndexerConstants.SPINDEXER_SIM_kV,
                IndexerConstants.SPINDEXER_SIM_kA));
            motorSim = spindexerMotor.getSimState();
            motorSim.setMotorType(MotorType.KrakenX44);
        }

        initLogging();
    }

    private void initLogging() {
        DataLog log = DataLogManager.getLog();

        transferLog = new DoubleLogEntry(log, "/Indexer/Transfer Velocity");
        spindexerLog = new DoubleLogEntry(log, "/Indexer/Spindexer Velocity");
    }

    @Override 
    public void periodic() {
        updateLogging();
    }

    private void updateLogging() {
        transferLog.append(transferMotor.getVelocity().getValueAsDouble());
        spindexerLog.append(spindexerMotor.getVelocity().getValueAsDouble());
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

    public double getTransferPower() {
        return transferDutyCycle.Output;
    }

    public double getSpindexerPower() {
        return spindexerDutyCycle.Output;
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

    public void setTransferVelocity(double velocity) {
        transferMotor.setControl(transferVelocityVoltage.withVelocity(velocity).withFeedForward(IndexerConstants.TRANSFER_kV_FEEDFORWARD));
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

    public void setPower(double spindexerPower, double transferPower) {
        setSpindexerPower(spindexerPower);
        setTransferPower(transferPower);
    }

    /**
     * stops all movement to the overall indexer
     */
    public void stop() {
        stopSpindexer();
        stopTransfer();
    }


    /**
     * dutycycleout command for indexer
     * @param power
     * @return the command for running the indexer
     */
    public Command indexCommand(double power) {
        return new StartEndCommand(() -> setPower(power), () -> stop(), this);
    }

    public Command indexCommand(DoubleSupplier spindexerPower, DoubleSupplier transferVelocity) {
        return new StartEndCommand(() -> setPower(spindexerPower.getAsDouble(), transferVelocity.getAsDouble()), this::stop);
    }

    public Command autoIndex(DoubleSupplier spindexerPower, DoubleSupplier transferVelocity) {
        return new InstantCommand(() -> setTransferVelocity(transferVelocity.getAsDouble()))
            .andThen(new WaitCommand(IndexerConstants.SPINDEXER_DELAY))
            .andThen(indexCommand(spindexerPower, transferVelocity));
    }

    public Command autoIndex(double spindexerPower, double transferVelocity) {
        return autoIndex(() -> spindexerPower, () -> transferVelocity);
    }

    public Command indexCommand(double spindexerPower, double transferVelocity) {
        return indexCommand(() -> spindexerPower, () -> transferVelocity);
    }

    public Command transferCommand(DoubleSupplier transferVelocity) {
        return new RunCommand(() -> setTransferVelocity(transferVelocity.getAsDouble()));
    }
}
