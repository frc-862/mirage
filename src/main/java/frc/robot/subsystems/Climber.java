// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.L3ClimbCommand;
import frc.robot.Robot;
import frc.robot.constants.RobotMap;
import frc.util.hardware.ThunderBird;
import frc.util.shuffleboard.LightningShuffleboard;

public class Climber extends SubsystemBase {

    public class ClimberConstants {
        public static final boolean CLIMBER_MOTOR_INVERTED = false; // temp
        public static final Current CLIMBER_MOTOR_STATOR_LIMIT = Amps.of(40); // temp
        public static final boolean CLIMBER_MOTOR_BRAKE_MODE = true; // temp

        public static final double GEARING_RATIO = 1d; // temp
        public static final Mass WEIGHT = Pounds.of(20); // temp
        public static final Distance LENGTH = Inches.of(0.5); // temp
        public static final Distance RADIUS = Inches.of(0.4); // temp
        public static final Distance MIN_HEIGHT = Inches.of(1); // temp
        public static final Distance MAX_HEIGHT = Inches.of(12); // temp
        public static final Distance START_HEIGHT  = Inches.of(3); // temp
    }

    private ThunderBird climberMotor;
    private DCMotor gearbox;
    private TalonFXSimState motorSim;
    private ElevatorSim climberSim;
    private DutyCycleOut dutyCycleOut;
    private DigitalInput forwardLimitSwitch;
    private DigitalInput reverseLimitSwitch;

    /** Creates a new Climber Subsystem. */
    public Climber() {
        climberMotor = new ThunderBird(RobotMap.CLIMBER, RobotMap.CAN_BUS,
            ClimberConstants.CLIMBER_MOTOR_INVERTED, ClimberConstants.CLIMBER_MOTOR_STATOR_LIMIT,
            ClimberConstants.CLIMBER_MOTOR_BRAKE_MODE);

        dutyCycleOut = new DutyCycleOut(0);

        forwardLimitSwitch = new DigitalInput(RobotMap.CLIMBER_FORWARD_LIMIT_SWITCH);
        reverseLimitSwitch = new DigitalInput(RobotMap.CLIMBER_REVERSE_LIMIT_SWITCH);

        if (Robot.isSimulation()){
            gearbox = DCMotor.getKrakenX60Foc(1);
            climberSim = new ElevatorSim(gearbox, ClimberConstants.GEARING_RATIO, ClimberConstants.WEIGHT.in(Kilograms),
            ClimberConstants.LENGTH.in(Meters), ClimberConstants.MIN_HEIGHT.in(Meters), ClimberConstants.MAX_HEIGHT.in(Meters),
             true, ClimberConstants.START_HEIGHT.in(Meters));
            motorSim = new TalonFXSimState(climberMotor);
        }
    }

    public void simulationPeriodic(){
        double batteryVoltage = RobotController.getBatteryVoltage();
        motorSim.setSupplyVoltage(batteryVoltage);

        climberSim.setInputVoltage(motorSim.getMotorVoltage());
        climberSim.update(Robot.kDefaultPeriod);
        motorSim.setRawRotorPosition(Units.metersToInches(climberSim.getPositionMeters()));
        LightningShuffleboard.getBool("Climber", "Climber Foward Limit Switch", false);
        LightningShuffleboard.getBool("Climber", "Climber Reverse Limit Switch", false);
    }

    /**
     * Set the climber's duty cycle output with the specified power.
     * @param power
     */
    public void setDutyCycle(double power) {
        climberMotor.setControl(dutyCycleOut.withOutput(power));
    }

    /**
     * Get the state of the forward limit switch.
     * @return true if the forward limit switch is triggered, otherwise returns false.
     */
    public boolean getForwardLimitSwitchState() {
       return forwardLimitSwitch.get();
    }

    /**
     * Get the state of the reverse limit switch.
     * @return true if the reverse limit switch is triggered, otherwise returns false.
     */
    public boolean getReverseLimitSwitchState() {
        return reverseLimitSwitch.get();
    }

    /**
     * Stop the climber motor.
     */
    public void stop() {
        climberMotor.stopMotor();
    }
    
    /**
     * The command extends the climber until it hits the forward limit switch, then reverses until the climber hits the reverse limit switch.
     * Repeats 3 times for a level 3 climb.
     * @return the level 3 climb command
     */
    public Command l3Climb() {
        return new L3ClimbCommand(this);
    }

    /**
     * The command extends the climber once until it hits the forward limit switch and then reverses for a little bit
     * NOTE: This command contains temporailly values for the reverse power & time.
     * @return The level 1 climb command
     */
    public Command l1Climb(){
        return new SequentialCommandGroup(
            new RunCommand(() -> setDutyCycle(1d)).until(() -> getForwardLimitSwitchState()),
            new RunCommand(() -> setDutyCycle(-1d)).withTimeout(2)
            .finallyDo(this::stop));
    }
}