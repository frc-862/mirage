// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.FlywheelConstants;
import frc.robot.constants.RobotMap;
import frc.util.hardware.ThunderBird;
import frc.util.shuffleboard.LightningShuffleboard;

public class Flywheel extends SubsystemBase {

    //creates a flywheel motor//
    private final ThunderBird flywheelMotor;

    private final DutyCycleOut dutyCycle;
    private final VelocityVoltage velocityPID;

    private AngularVelocity targetVelocity;

    private TalonFXSimState motorSim;
    private FlywheelSim shooterSim;

    /** Creates a new Flywheel Subsystem. */
    public Flywheel() {
        this(new ThunderBird(RobotMap.FLYWHEEL_MOTOR_ID, RobotMap.CAN_BUS,
            FlywheelConstants.INVERTED, FlywheelConstants.STATOR_LIMIT, FlywheelConstants.BRAKE));
    }

    /**
     * Creates a new Flywheel Subsystem with a provided motor.
     * @param motor the ThunderBird motor to use for the flywheel
     */
    public Flywheel(ThunderBird motor) {
        //Sets the flywheel motor
        flywheelMotor = motor;

        //instatiates duty cycle and velocity pid
        dutyCycle = new DutyCycleOut(0.0);
        velocityPID = new VelocityVoltage(0d);

        //creates a config for the flywheel motor
        TalonFXConfiguration config = flywheelMotor.getConfig();

        config.Slot0.kP = FlywheelConstants.kP;
        config.Slot0.kI = FlywheelConstants.kI;
        config.Slot0.kD = FlywheelConstants.kD;
        config.Slot0.kV = FlywheelConstants.kV;
        config.Slot0.kS = FlywheelConstants.kS;

        config.Feedback.SensorToMechanismRatio = FlywheelConstants.GEAR_RATIO;

        flywheelMotor.applyConfig(config);


        if (Robot.isSimulation()){
            motorSim = flywheelMotor.getSimState();
            motorSim.setMotorType(MotorType.KrakenX60);

            shooterSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60Foc(1), FlywheelConstants.MOI.in(KilogramSquareMeters),
                FlywheelConstants.GEAR_RATIO), DCMotor.getKrakenX60Foc(1));
        }
    }

    @Override
    public void simulationPeriodic() {
        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        shooterSim.setInput(motorSim.getMotorVoltageMeasure().in(Volts));
        shooterSim.update(Robot.kDefaultPeriod);

        motorSim.setRotorVelocity(shooterSim.getAngularVelocity());

        LightningShuffleboard.setDouble("Flywheel", "Velocity", getVelocity().in(RotationsPerSecond));
    }

    /**
     * Sets motor power of the flywheel
     * @param power
     */
    public void setPower(double power) {
        flywheelMotor.setControl(dutyCycle.withOutput(power));
    }

    /**
     * Stops all movement of the flywheel motor
     */
    public void stopMotor() {
        flywheelMotor.stopMotor();
    }

    /**
     * set motor velocity
     * @param velocity
     */
    public void setVelocity(AngularVelocity velocity){
        targetVelocity = velocity;
        flywheelMotor.setControl(velocityPID.withVelocity(velocity));
    }

    /**
     * @return the velocity of the flywheel motor
     */
    public AngularVelocity getVelocity(){
        return flywheelMotor.getVelocity().getValue();
    }

    /**
     * @return whether or not the current velocity is near the target velocity
     */
    public boolean velocityOnTarget(){
        return getVelocity().isNear(targetVelocity, FlywheelConstants.TOLERANCE);
    }

}
