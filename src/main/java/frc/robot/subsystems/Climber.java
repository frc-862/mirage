// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;

import org.dyn4j.dynamics.Settings;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.RobotMap;
import frc.util.hardware.ThunderBird;

public class Climber extends SubsystemBase {
    private ThunderBird climberMotor;
    private CANcoder encoder;
    private final DutyCycleOut dutyCycle;
    private final PositionVoltage positionPID;
    private DCMotor gearbox;
    private TalonFXSimState motorSim;
    private ElevatorSim climberSim;
    private CANcoderSimState encoderSim;

    /** Creates a new Climber Subsystem. */
    public Climber() {
        climberMotor = new ThunderBird(RobotMap.CLIMBER, RobotMap.CAN_BUS,
            ClimberConstants.CLIMBER_MOTOR_INVERTED, ClimberConstants.CLIMBER_MOTOR_STATOR_LIMIT,
            ClimberConstants.CLIMBER_MOTOR_BRAKE_MODE);
        encoder = new CANcoder(RobotMap.CLIMBER_ENCODER);

        dutyCycle = new DutyCycleOut(0.0);
        positionPID = new PositionVoltage(0);

        TalonFXConfiguration config = climberMotor.getConfig();
        config.Slot0.kP = ClimberConstants.CLIMB_KP;
        config.Slot0.kI = ClimberConstants.CLIMB_KI;
        config.Slot0.kD = ClimberConstants.CLIMB_KD;
        config.Slot0.kS = ClimberConstants.CLIMB_KS;
        config.Slot0.kV = ClimberConstants.CLIMB_KV;
        config.Slot0.kA = ClimberConstants.CLIMB_KA;
        config.Slot0.kG = ClimberConstants.CLIMB_KG;
        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        if (Robot.isSimulation()){
            gearbox = DCMotor.getKrakenX60Foc(1);
            climberSim = new ElevatorSim(gearbox, ClimberConstants.GEARING_RATIO, ClimberConstants.WEIGHT.in(Pounds),
            ClimberConstants.LENGTH.in(Meter), ClimberConstants.MIN_HEIGHT.in(Meter), ClimberConstants.MAX_HEIGHT.in(Meter),
             true, ClimberConstants.START_HEIGHT.in(Meter), ClimberConstants.STANDARD_DEVIATION);
            motorSim = new TalonFXSimState(climberMotor);
            encoderSim = new CANcoderSimState(encoder);
        }
    }

    @Override
    public void periodic(){}

    public void simulationPeriodic(){
        double batteryVoltage = RobotController.getBatteryVoltage();
        motorSim.setSupplyVoltage(batteryVoltage);
        encoderSim.setSupplyVoltage(batteryVoltage);

        climberSim.setInputVoltage(motorSim.getMotorVoltage());
        climberSim.update(Robot.kDefaultPeriod);
        motorSim.setRawRotorPosition(Units.metersToInches(climberSim.getPositionMeters()));

    }

    /**
     * Sets the power output of the climber motor.
     *
     * @param power the power output as a value between -1.0 and 1.0
     */
    public void setPower(double power) {
        dutyCycle.Output = power;
        climberMotor.setControl(dutyCycle);
    }

    /**
     * Sets the target position of the climber motor.
     *
     * @param positionVolts the target position in volts
     */
    public void setPosition(double positionVolts) {
        climberMotor.setControl(positionPID.withPosition(positionVolts));
    }

    /**
     * gets the position of the climber
     * @return climber position
     */
    public double getTargetPosition() {
        return positionPID.Position;
    }

    /**
     * gets if the climber is on target
     * @return if climber is on target
     */
    public boolean onTarget() {
        return positionPID.getPositionMeasure().isNear(climberMotor.getPosition().getValue(), ClimberConstants.CLIMB_TOLERANCE);
    }
}
