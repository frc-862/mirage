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
import frc.robot.constants.ShooterConstants;
import frc.robot.Robot;
import frc.robot.constants.RobotMap;
import frc.util.hardware.ThunderBird;
import frc.util.shuffleboard.LightningShuffleboard;

public class Shooter extends SubsystemBase {

    //creates a shooter motor//
    private ThunderBird shooterMotor;

    private final DutyCycleOut dutyCycle;
    private final VelocityVoltage velocityPID;

    private AngularVelocity targetVelocity;

    private TalonFXSimState motorSim;
    private FlywheelSim shooterSim;

    /** Creates a new Shooter Subsystem. */
    public Shooter() {
        this(new ThunderBird(RobotMap.SHOOTER_MOTOR_ID, RobotMap.CAN_BUS,
            ShooterConstants.SHOOTER_MOTOR_INVERTED, ShooterConstants.SHOOTER_MOTOR_STATOR_LIMIT,
            ShooterConstants.SHOOTER_MOTOR_BRAKE));
    }

    /** Creates a new Shooter Subsystem. */
    public Shooter(ThunderBird motor) {
        //Sets new motors
        shooterMotor = motor;

        //instatiates duty cycle and velocity pid
        dutyCycle = new DutyCycleOut(0.0);
        velocityPID = new VelocityVoltage(0d);

        //creates a config for the shooter motor
        TalonFXConfiguration shooterMotorConfig = shooterMotor.getConfig();
        shooterMotorConfig.Slot0.kP = ShooterConstants.kP;
        shooterMotorConfig.Slot0.kI = ShooterConstants.kI;
        shooterMotorConfig.Slot0.kD = ShooterConstants.kD;
        shooterMotorConfig.Slot0.kV = ShooterConstants.kV;
        shooterMotorConfig.Slot0.kS = ShooterConstants.kS;
        shooterMotor.applyConfig(shooterMotorConfig);

        if (Robot.isSimulation()){
            motorSim = shooterMotor.getSimState();
            motorSim.setMotorType(MotorType.KrakenX60);

            shooterSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60Foc(1), ShooterConstants.MOI.in(KilogramSquareMeters),
                ShooterConstants.GEAR_RATIO), DCMotor.getKrakenX60Foc(1));
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
        shooterMotor.setControl(dutyCycle.withOutput(power));
    }

    /**
     * Stops all movement of the shooter motor
     */
    public void stopMotor() {
        shooterMotor.stopMotor();
    }

    /**
     * set motor velocity
     * @param velocity
     */
    public void setVelocity(AngularVelocity velocity){
        targetVelocity = velocity;
        shooterMotor.setControl(velocityPID.withVelocity(velocity));
    }

    /**
     * @return the velocity of the shooter motor
     */
    public AngularVelocity getVelocity(){
        return shooterMotor.getVelocity().getValue();
    }

    /**
     * @return whether or not the current velocity is near the target velocity
     */
    public boolean velocityOnTarget(){
        return getVelocity().isNear(targetVelocity, ShooterConstants.TOLERANCE);
    }

}
