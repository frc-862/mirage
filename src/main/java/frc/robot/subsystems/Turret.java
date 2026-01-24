// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.RobotMap;
import frc.robot.constants.TurretConstants;
import frc.util.hardware.ThunderBird;
import frc.util.shuffleboard.LightningShuffleboard;

public class Turret extends SubsystemBase {
    private ThunderBird motor;
    private CANcoder encoder;

    private Angle targetPosition = Rotations.zero();

    public final PositionVoltage positionPID = new PositionVoltage(0);

    private DCMotor gearbox;
    private SingleJointedArmSim turretSim;
    private TalonFXSimState motorSim;
    private CANcoderSimState encoderSim;

    /** Creates a new TurretAim. */
    public Turret() {
        motor = new ThunderBird(RobotMap.TURRET_MOTOR_ID, RobotMap.CAN_BUS, RobotMap.TURRET_MOTOR_INVERTED, RobotMap.TURRET_STATOR_LIMIT, RobotMap.TURRET_BRAKE);
        encoder = new CANcoder(RobotMap.TURRET_ENCODER_ID, RobotMap.CAN_BUS);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        CANcoderConfiguration angleConfig = new CANcoderConfiguration();
        angleConfig.MagnetSensor.MagnetOffset = Robot.isReal() ? TurretConstants.OFFSET : 0;
        angleConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        encoder.getConfigurator().apply(angleConfig);

        motorConfig.Slot0.kP = TurretConstants.MOTOR_KP;
        motorConfig.Slot0.kI = TurretConstants.MOTOR_KI;
        motorConfig.Slot0.kD = TurretConstants.MOTOR_KD;
        motorConfig.Slot0.kS = TurretConstants.MOTOR_KS;
        motorConfig.Slot0.kV = TurretConstants.MOTOR_KV;
        motorConfig.Slot0.kA = TurretConstants.MOTOR_KA;
        motorConfig.Slot0.kG = TurretConstants.MOTOR_KG;

        motorConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        motorConfig.Feedback.SensorToMechanismRatio = TurretConstants.ENCODER_TO_MECHANISM_RATIO;
        motorConfig.Feedback.RotorToSensorRatio = TurretConstants.ROTOR_TO_ENCODER_RATIO;

        motor.applyConfig(motorConfig);

        if (Robot.isSimulation()) {
            gearbox = DCMotor.getKrakenX44Foc(1);
            turretSim = new SingleJointedArmSim(gearbox, TurretConstants.ROTOR_TO_ENCODER_RATIO,
                TurretConstants.MOI.magnitude(), TurretConstants.LENGTH.in(Meters),
                TurretConstants.MIN_ANGLE.in(Radians), TurretConstants.MAX_ANGLE.in(Radians),
                false, TurretConstants.MIN_ANGLE.in(Radians), 0d, 1d);

            motorSim = new TalonFXSimState(motor);
            encoderSim = new CANcoderSimState(encoder);

            motorSim.setRawRotorPosition(TurretConstants.MIN_ANGLE.in(Rotations));
            encoderSim.setRawPosition(TurretConstants.MIN_ANGLE.in(Rotations));
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    /**
     * @param angle sets the angle to the motor of the turret
     */
    public void setAngle(Angle angle) {
        targetPosition = angle;
        motor.setControl(positionPID.withPosition(targetPosition));
    }

    public Angle getAngle() {
        return encoder.getAbsolutePosition().getValue();
    }

    public Angle getTargetAngle() {
        return targetPosition;
    }

    public boolean isOnTarget() {
        return MathUtil.isNear(getTargetAngle().in(Degree), getAngle().in(Degree), TurretConstants.ANGLE_TOLERANCE);
    }

    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void simulationPeriodic() {
        double batteryVoltage = RobotController.getBatteryVoltage();
        motorSim.setSupplyVoltage(batteryVoltage);
        encoderSim.setSupplyVoltage(batteryVoltage);

        turretSim.setInputVoltage(motorSim.getMotorVoltage());
        turretSim.update(Robot.kDefaultPeriod);

        Angle simAngle = Rotations.of(turretSim.getAngleRads());
        AngularVelocity simVeloc = RadiansPerSecond.of(turretSim.getVelocityRadPerSec());
        motorSim.setRawRotorPosition(simAngle);
        motorSim.setRotorVelocity(simVeloc);
        encoderSim.setRawPosition(simAngle);
        encoderSim.setVelocity(simVeloc);

        LightningShuffleboard.setDouble("Turret", "CANcoder angle", encoder.getAbsolutePosition().getValue().in(Degree));
        LightningShuffleboard.setDouble("Turret", "sim angle", simAngle.in(Degree));
        LightningShuffleboard.setDouble("Turret", "getPose", getAngle().in(Degree));

        LightningShuffleboard.setDouble("Turret", "current angle", getAngle().in(Degree));
        LightningShuffleboard.setDouble("Turret", "target angle", getTargetAngle().in(Degree));
        LightningShuffleboard.setBool("Turret", "on target", isOnTarget());
    }
}