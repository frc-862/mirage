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
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.RobotMap;
import frc.robot.constants.TurretConstants;
import frc.util.hardware.ThunderBird;
import frc.util.shuffleboard.LightningShuffleboard;

public class Turret extends SubsystemBase {
  ThunderBird motor;
  CANcoder encoder;

  private double targetPosition = 0;

  public final PositionVoltage positionPID = new PositionVoltage(0);

  private DCMotor gearbox;
  private SingleJointedArmSim turretSim;
  private TalonFXSimState motorSim;
  private CANcoderSimState encoderSim;

  private Mechanism2d mech2d;
  private MechanismLigament2d turretLigament;

  /** Creates a new TurretAim. */
  public Turret() {
    // CREATE CONSTANTS FOR THIS
    motor = new ThunderBird(0, "canivore", false, 0, false);
    encoder = new CANcoder(0, "canivore");

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    CANcoderConfiguration angleConfig = new CANcoderConfiguration();
    angleConfig.MagnetSensor.MagnetOffset = Robot.isReal() ? TurretConstants.turretOffset : 0;
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
      gearbox = DCMotor.getKrakenX60(1);
      turretSim = new SingleJointedArmSim(gearbox, TurretConstants.ROTOR_TO_ENCODER_RATIO,
          TurretConstants.MOI.magnitude(), TurretConstants.LENGTH.in(Meter),
          TurretConstants.MIN_ANGLE.in(Radian), TurretConstants.MAX_ANGLE.in(Radian), false, 
          TurretConstants.MIN_ANGLE.in(Radian), 0d, 1d);
      
      motorSim = new TalonFXSimState(motor);
      encoderSim = new CANcoderSimState(encoder);
 
      encoderSim.setRawPosition(TurretConstants.MIN_ANGLE.in(Rotations));
      motorSim.setRawRotorPosition(TurretConstants.MIN_ANGLE.in(Rotations));

      mech2d = new Mechanism2d(1, 1);
      MechanismRoot2d root = mech2d.getRoot("turret", 0.5, 0.5);
      turretLigament = new MechanismLigament2d("turret", 0.4, 0);
      root.append(turretLigament);

      SmartDashboard.putData("Turret Sim", mech2d);
      SmartDashboard.putNumber("Test Value", 42);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setAngle(double position) {
    targetPosition = MathUtil.clamp(position, -220, 220);
    motor.setControl(positionPID.withPosition(targetPosition));
  }

  public double getAngle() {
    return encoder.getAbsolutePosition().getValue().in(Rotations);
  }

  public double getTargetAngle() {
    return targetPosition;
  }

  public boolean isOnTarget() {
    return Math.abs(getAngle() - getTargetAngle()) < TurretConstants.TURRET_ANGLE_TOLERANCE;
  }

  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void simulationPeriodic() {
    double batteryVoltage = RobotController.getBatteryVoltage();
    motorSim.setSupplyVoltage(batteryVoltage);
    encoderSim.setSupplyVoltage(batteryVoltage);

    double simAngle = Radian.of(turretSim.getAngleRads()).in(Rotations);
    double simVeloc = Radian.of(turretSim.getVelocityRadPerSec()).in(Rotations) * TurretConstants.ROTOR_TO_ENCODER_RATIO;
    motorSim.setRawRotorPosition(simAngle);
    motorSim.setRotorVelocity(simVeloc);
    encoderSim.setRawPosition(simAngle);
    encoderSim.setVelocity(simVeloc);

    turretLigament.setAngle(getAngle());

    LightningShuffleboard.setDouble("Turret", "CANCoder angle", encoder.getAbsolutePosition().getValue().in(Degree));
    LightningShuffleboard.setDouble("Turret", "Sim Angle", simAngle);
    LightningShuffleboard.setDouble("Turret", "getPose", getAngle());

    turretSim.setInputVoltage(motorSim.getMotorVoltage());
    turretSim.update(RobotMap.UPDATE_FREQ);

    LightningShuffleboard.setDouble("Turret", "current angle", getAngle());
    LightningShuffleboard.setDouble("Turret", "target angle", getTargetAngle());
    LightningShuffleboard.setBool("Turret", "on target", isOnTarget());
  }
}
