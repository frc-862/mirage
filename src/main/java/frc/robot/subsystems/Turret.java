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

import edu.wpi.first.math.MathUtil;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.TurretConstants;
import frc.util.hardware.ThunderBird;

public class Turret extends SubsystemBase {
  ThunderBird motor;
  CANcoder encoder;
  private double targetPosition = 0;

  public final PositionVoltage positionPID = new PositionVoltage(0);

  /** Creates a new TurretAim. */
  public Turret() {
    // CREATE CONSTANTS FOR THIS
    motor = new ThunderBird(0, "canivore", false, 0, false);
    encoder = new CANcoder(0, "canivore");

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    CANcoderConfiguration angleConfig = new CANcoderConfiguration();
    angleConfig.MagnetSensor.MagnetOffset = Robot.isReal() ? TurretConstants.wristOffset : 0;
    angleConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoder.getConfigurator().apply(angleConfig);

    motorConfig.Slot0.kP = TurretConstants.MOTOR_KP;
    motorConfig.Slot0.kI = TurretConstants.MOTOR_KI;
    motorConfig.Slot0.kD = TurretConstants.MOTOR_KP;
    motorConfig.Slot0.kS = TurretConstants.MOTOR_KI;
    motorConfig.Slot0.kV = TurretConstants.MOTOR_KP;
    motorConfig.Slot0.kA = TurretConstants.MOTOR_KI;
    motorConfig.Slot0.kG = TurretConstants.MOTOR_KG;

    motorConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
  
    motorConfig.Feedback.SensorToMechanismRatio = TurretConstants.ENCODER_TO_MECHANISM_RATIO;
    motorConfig.Feedback.RotorToSensorRatio = TurretConstants.ROTOR_TO_ENCODER_RATIO;

    motor.applyConfig(motorConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setAngle(double position) {
    targetPosition = MathUtil.clamp(position, -220, 220);
    Angle targetAngle = Degree.of(targetPosition);
    motor.setControl(positionPID.withPosition(targetAngle.in(Rotations)));
  }

  public double getAngle() {
    return encoder.getAbsolutePosition().getValue().in(Degree);
  }

  public double getTargetAngle() {
    return targetPosition;
  }

  public boolean isOnTarget() {
    return Math.abs(getAngle() - getTargetAngle()) < TurretConstants.TURRET_ANGLE_TOLERANCE;
  }

  public void stop() {

  }
}
