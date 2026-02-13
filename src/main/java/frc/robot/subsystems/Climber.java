// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pound;
import static edu.wpi.first.units.Units.Pounds;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.RobotMap;
import frc.util.hardware.ThunderBird;

public class Climber extends SubsystemBase {

    public class ClimberConstants {
        public static final boolean CLIMBER_MOTOR_INVERTED = false; // temp
        public static final Current CLIMBER_MOTOR_STATOR_LIMIT = Amps.of(40); // temp
        public static final boolean CLIMBER_MOTOR_BRAKE_MODE = false; // temp

        public static final double CLIMB_KP = 0.0; // temp
        public static final double CLIMB_KI = 0.0; // temp
        public static final double CLIMB_KD = 0.0; // temp
        public static final double CLIMB_KS = 0.0; // temp
        public static final double CLIMB_KV = 0.0; // temp
        public static final double CLIMB_KA = 0.0; // temp
        public static final double CLIMB_KG = 0.0; // temp

        public static final double CLIMB_TOLERANCE = 0.5;

        public static final double GEARING_RATIO = 1d; // temp
        public static final Mass WEIGHT = Pound.of(20); // temp
        public static final Distance LENGTH = Inches.of(0.5); // temp
        public static final Distance RADIUS = Inches.of(0.4); // temp
        public static final Distance MIN_HEIGHT = Inches.of(1); // temp
        public static final Distance MAX_HEIGHT = Inches.of(12); // temp
        public static final Distance START_HEIGHT  = Inches.of(3); // temp
        public static final double  STANDARD_DEVIATION = 0d; // temp
    }

    private ThunderBird climberMotor;
    private CANcoder encoder;
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
            ClimberConstants.LENGTH.in(Inches), ClimberConstants.MIN_HEIGHT.in(Inches), ClimberConstants.MAX_HEIGHT.in(Inches),
             true, ClimberConstants.START_HEIGHT.in(Inches), ClimberConstants.STANDARD_DEVIATION);
            motorSim = new TalonFXSimState(climberMotor);
            encoderSim = new CANcoderSimState(encoder);
        }
    }

    public void simulationPeriodic(){
        double batteryVoltage = RobotController.getBatteryVoltage();
        motorSim.setSupplyVoltage(batteryVoltage);
        encoderSim.setSupplyVoltage(batteryVoltage);

        climberSim.setInputVoltage(motorSim.getMotorVoltage());
        climberSim.update(Robot.kDefaultPeriod);
        motorSim.setRawRotorPosition(Units.metersToInches(climberSim.getPositionMeters()));
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
