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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.RobotMap;
import frc.util.hardware.ThunderBird;
import frc.util.shuffleboard.LightningShuffleboard;

public class Shooter extends SubsystemBase {
    private final ThunderBird motor;

    private final DutyCycleOut dutyCycle;
    private final VelocityVoltage velocityPID;

    private AngularVelocity targetVelocity;

    private TalonFXSimState motorSim;
    private FlywheelSim shooterSim;

    /** Creates a new Shooter Subsystem. */
    public Shooter() {
        this(new ThunderBird(RobotMap.SHOOTER, RobotMap.CAN_BUS,
            ShooterConstants.INVERTED, ShooterConstants.STATOR_LIMIT, ShooterConstants.BRAKE));
    }

    /**
     * Creates a new Shooter Subsystem.
     * @param motor the ThunderBird motor to use for the shooter
     */
    public Shooter(ThunderBird motor) {
        this.motor = motor;

        //instatiates duty cycle and velocity pid
        dutyCycle = new DutyCycleOut(0.0);
        velocityPID = new VelocityVoltage(0d);

        //creates a config for the shooter motor
        TalonFXConfiguration config = motor.getConfig();

        config.Slot0.kP = ShooterConstants.kP;
        config.Slot0.kI = ShooterConstants.kI;
        config.Slot0.kD = ShooterConstants.kD;
        config.Slot0.kV = ShooterConstants.kV;
        config.Slot0.kS = ShooterConstants.kS;

        config.Feedback.SensorToMechanismRatio = ShooterConstants.GEAR_RATIO;

        motor.applyConfig(config);


        if (Robot.isSimulation()){
            motorSim = motor.getSimState();
            motorSim.setMotorType(MotorType.KrakenX60);

            shooterSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60Foc(1), ShooterConstants.MOI.in(KilogramSquareMeters),
                ShooterConstants.GEAR_RATIO), DCMotor.getKrakenX60Foc(1));
        }
    }

    @Override
    public void periodic() {}

    @Override
    public void simulationPeriodic() {
        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        shooterSim.setInput(motorSim.getMotorVoltageMeasure().in(Volts));
        shooterSim.update(Robot.kDefaultPeriod);

        motorSim.setRotorVelocity(shooterSim.getAngularVelocity());

        LightningShuffleboard.setDouble("Shooter", "Velocity", getVelocity().in(RotationsPerSecond));
    }

    /**
     * Set the power of the shooter motor using duty cycle out
     * @param power duty cycle value from -1.0 to 1.0
     */
    public void setPower(double power) {
        motor.setControl(dutyCycle.withOutput(power));
    }

    /**
     * stops all movement to the shooter motor
     */
    public void stopMotor() {
        motor.stopMotor();
    }

    /**
     * set motor velocity
     * @param velocity
     */
    public void setVelocity(AngularVelocity velocity){
        targetVelocity = velocity;
        motor.setControl(velocityPID.withVelocity(velocity));
    }

    /**
     * @return the velocity of the shooter motor
     */
    public AngularVelocity getVelocity(){
        return motor.getVelocity().getValue();
    }

    /**
     * @return whether or not the current velocity is near the target velocity
     */
    public boolean velocityOnTarget(){
        return getVelocity().isNear(targetVelocity, ShooterConstants.TOLERANCE);
    }

    public Command Shoot(double power) {
        Command shoot = new Command() {

            @Override
            public void initialize() {
                setPower(power);
            }

            @Override
            public void end(boolean interrupted) {
                stopMotor();
            }
        };

        return shoot;
    }
}
