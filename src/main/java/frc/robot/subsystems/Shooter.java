// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.RobotMap;
import frc.util.hardware.ThunderBird;
import frc.util.shuffleboard.LightningShuffleboard;
import frc.util.units.ThunderMap;

public class Shooter extends SubsystemBase {

    public class ShooterConstants {
        public static final boolean INVERTED = false; // temp
        public static final Current STATOR_LIMIT = Amps.of(160.0); // temp
        public static final boolean BRAKE = false; // temp
        public static final double COAST_DC = 0.05; // Shooter power when coasting

        public static final double kP = 0.75d;
        public static final double kI = 0d;
        public static final double kD = 0d;
        public static final double kV = 0.1185d;
        public static final double kS = 0.38d;
        public static final AngularVelocity TOLERANCE = RotationsPerSecond.of(2);
        public static final AngularVelocity BIAS_DELTA = RotationsPerSecond.of(1);

        public static final double GEAR_RATIO = 1d; // temp
        public static final Distance FLYWHEEL_CIRCUMFERENCE = Inches.of(4).times(Math.PI);

        // for sim to account for movement between shooter, fuel, and hood
        public static final double SHOOTER_EFFICIENCY = 0.3; 
        // Input is distance to target in meters, output is shooter speed in rotations per second
        public static final ThunderMap<Distance, AngularVelocity> VELOCITY_MAP = new ThunderMap<>() {
            {
                put(Meters.of(2d), RotationsPerSecond.of(20d));
                put(Meters.of(4d), RotationsPerSecond.of(40d));
                put(Meters.of(6d), RotationsPerSecond.of(60d));
            }
        };

        // Sim
        public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.05); // temp
        public static final Translation2d SHOOTER_POSITION_ON_ROBOT = new Translation2d(Inches.of(7), Inches.of(7)); // temp
        public static final Distance SHOOTER_HEIGHT = Inches.of(18);
        public static final Time MAX_SHOOTING_PERIOD = Seconds.of(0.1); // 10 balls per second
    }

    private final ThunderBird motorLeft;
    private final ThunderBird motorRight;

    private final DutyCycleOut dutyCycle;
    private final VelocityVoltage velocityPID;

    private AngularVelocity targetVelocity;


    private MutAngularVelocity shooterBias;

    private TalonFXSimState motorSim;
    private FlywheelSim shooterSim;

    /** Creates a new Shooter Subsystem. */
    public Shooter() {
        this(new ThunderBird(RobotMap.SHOOTER_LEFT, RobotMap.CAN_BUS,
            ShooterConstants.INVERTED, ShooterConstants.STATOR_LIMIT, ShooterConstants.BRAKE),
            new ThunderBird(RobotMap.SHOOTER_RIGHT, RobotMap.CAN_BUS,
            ShooterConstants.INVERTED, ShooterConstants.STATOR_LIMIT, ShooterConstants.BRAKE));
    }

    /**
     * Creates a new Shooter Subsystem.
     * @param motorLeft the ThunderBird motor to use for the shooter
     * @param motorRight the ThunderBird motor that follows for the shooter
     */
    public Shooter(ThunderBird motorLeft, ThunderBird motorRight) {
        this.motorLeft = motorLeft;
        this.motorRight = motorRight;

        //instatiates duty cycle and velocity pid
        dutyCycle = new DutyCycleOut(0.0);
        velocityPID = new VelocityVoltage(0d);
        shooterBias = RotationsPerSecond.mutable(0);
        targetVelocity = RotationsPerSecond.zero();

        //creates a config for the shooter motor
        TalonFXConfiguration config = motorLeft.getConfig();

        config.Slot0.kP = ShooterConstants.kP;
        config.Slot0.kI = ShooterConstants.kI;
        config.Slot0.kD = ShooterConstants.kD;
        config.Slot0.kV = ShooterConstants.kV;
        config.Slot0.kS = ShooterConstants.kS;

        config.Feedback.SensorToMechanismRatio = ShooterConstants.GEAR_RATIO;

        motorLeft.applyConfig(config);
        motorRight.setControl(new Follower(RobotMap.SHOOTER_LEFT, MotorAlignmentValue.Opposed));

        if (Robot.isSimulation()){
            motorSim = motorLeft.getSimState();
            motorSim.setMotorType(MotorType.KrakenX60);

            shooterSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60Foc(2), ShooterConstants.MOI.in(KilogramSquareMeters),
                ShooterConstants.GEAR_RATIO), DCMotor.getKrakenX60Foc(2));
        }
    }

    @Override
    public void simulationPeriodic() {
        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        shooterSim.setInput(motorSim.getMotorVoltageMeasure().in(Volts));
        shooterSim.update(Robot.kDefaultPeriod);

        motorSim.setRotorVelocity(shooterSim.getAngularVelocity());

        LightningShuffleboard.setDouble("Shooter", "Left Velocity", getLeftVelocity().in(RotationsPerSecond));
        LightningShuffleboard.setDouble("Shooter", "Right Velocity", getRightVelocity().in(RotationsPerSecond));
        LightningShuffleboard.setDouble("Shooter", "Target Velocity", targetVelocity.in(RotationsPerSecond));
        LightningShuffleboard.setDouble("Shooter", "Bias", shooterBias.in(RotationsPerSecond));
    }

     /**
     * Set the power of the shooter motor using duty cycle out
     * @param power duty cycle value from -1.0 to 1.0
     */
    public void setPower(double power) {
        motorLeft.setControl(dutyCycle.withOutput(power));
        targetVelocity = RotationsPerSecond.zero();
    }

    /**
     * stops all movement to the shooter motor
     */
    public void stop() {
        motorLeft.stopMotor();
        targetVelocity = RotationsPerSecond.zero();
    }

    /**
     * set motor velocity
     * @param velocity
     */
    public void setVelocity(AngularVelocity velocity){
        targetVelocity = velocity;
        applyChange();
    }

    /**
     * Change the bias of the shooter velocity. If the shooter is currently running it will update the velocity with the bias.
     * @param bias in rotations per second
     */
    public void changeBias(AngularVelocity bias) {
        shooterBias.mut_plus(bias);
        if (targetVelocity.gt(RotationsPerSecond.zero())) {
            applyChange();
        }
    }

    public void setBias(AngularVelocity bias) {
        shooterBias.mut_replace(bias);
        if (targetVelocity.gt(RotationsPerSecond.zero())) {
            applyChange();
        }        
    }
    
    private void applyChange() {
        motorLeft.setControl(velocityPID.withVelocity(getTargetVelocityWithBias().in(RotationsPerSecond)));
    }

    /**
     * Get the current bias of the shooter velocity.
     * @return the current bias in rotations per second
     */
    public AngularVelocity getBias() {
        return shooterBias;
    } 
    /**
     * @return the velocity of the shooter motor
     */
    public AngularVelocity getLeftVelocity(){
        return motorLeft.getVelocity().getValue();
    }

    public AngularVelocity getRightVelocity(){
        return motorRight.getVelocity().getValue();
    }

    public AngularVelocity getTargetVelocity() {
        return targetVelocity;
    }
    
    public AngularVelocity getTargetVelocityWithBias() {
        return targetVelocity.plus(shooterBias);
    }
    /**
     * @return whether or not the current velocity is near the target velocity
     */
    public boolean isOnTarget(){
        return getLeftVelocity().isNear(getTargetVelocityWithBias(), ShooterConstants.TOLERANCE);
    }

    /**
     * velocity control command for shooter
     * @param velocity
     * @return the command for running the shooter
     */
    public Command shootCommand(AngularVelocity velocity) {
        return shootCommand(() -> velocity);
    }

    /**
     * velocity control command for shooter
     * @param velocitySupplier
     * @return the command for running the shooter
     */
    public Command shootCommand(Supplier<AngularVelocity> velocitySupplier) {
        return new StartEndCommand(() -> setVelocity(velocitySupplier.get()), () -> {}, this).until(this::isOnTarget);
    }

    /**
     * Sets shooter motor into an idle power
     * @return the command for running the shooter at coast power
     */
    public Command coast() {
        return new InstantCommand(() -> setPower(ShooterConstants.COAST_DC), this);
    }
}
