// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;


import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.Units;
import frc.util.hardware.ThunderBird;
import frc.util.shuffleboard.LightningShuffleboard;
import frc.robot.constants.RobotMap;
import frc.robot.Robot;
import frc.robot.constants.HoodConstants;

public class Hood extends SubsystemBase {
    private ThunderBird hoodMotor;
    private CANcoder encoder;

    public final DutyCycleOut dutyCycle;

    final PositionVoltage request;
    private Angle targetAngle;

    private DCMotorSim hoodSim;
    private TalonFXSimState motorSim;
    private DCMotor gearbox;
    private MechanismLigament2d ligament;
    private MechanismRoot2d root2d;
    private Mechanism2d mech2d;
    private CANcoderSimState encoderSim;

    /** Creates a new Hood Subsystem. */
    public Hood() {
        hoodMotor = new ThunderBird(RobotMap.HOOD, RobotMap.CAN_BUS,
            HoodConstants.INVERTED, HoodConstants.STATOR_LIMIT,
            HoodConstants.BRAKE);
        encoder = new CANcoder(RobotMap.HOOD_ENCODER, RobotMap.CAN_BUS);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        CANcoderConfiguration angleConfig = new CANcoderConfiguration();

        dutyCycle = new DutyCycleOut(0d);

        request = new PositionVoltage(0d);

        encoder.getConfigurator().apply(angleConfig);

        motorConfig.Slot0.kP = HoodConstants.kP;
        motorConfig.Slot0.kI = HoodConstants.kI;
        motorConfig.Slot0.kD = HoodConstants.kD;
        motorConfig.Slot0.kS = HoodConstants.kS;
        motorConfig.Slot0.kV = HoodConstants.kV;
        motorConfig.Slot0.kA = HoodConstants.kA;
        targetAngle = Degrees.of(0);

        motorConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        motorConfig.Feedback.SensorToMechanismRatio = HoodConstants.ENCODER_TO_MECHANISM_RATIO;
        motorConfig.Feedback.RotorToSensorRatio = HoodConstants.ROTOR_TO_ENCODER_RATIO;
        hoodMotor.applyConfig(motorConfig);

        if (Robot.isSimulation()) {
            gearbox = DCMotor.getKrakenX44Foc(1);
            hoodSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(gearbox, HoodConstants.MOI.magnitude(), HoodConstants.GEARING_RATIO),
                gearbox
            );

            motorSim = new TalonFXSimState(hoodMotor);
            encoderSim = new CANcoderSimState(encoder);

            motorSim.setRawRotorPosition(HoodConstants.MIN_ANGLE.in(Rotations));
            encoderSim.setRawPosition(HoodConstants.MIN_ANGLE.in(Rotations));

            mech2d = new Mechanism2d(3,  3);
            root2d =  mech2d.getRoot("Hood", 2, 0);

            ligament = root2d.append(new MechanismLigament2d("Hood", 3, 90));
            LightningShuffleboard.send("Hood", "Mech2d", mech2d);
        }
    }

    @Override
    public void periodic() {}

    /**
     * Sets position of the hood
     * @param position in degrees
     */
    public void setPosition(Angle position) {
        targetAngle = Units.clamp(position, HoodConstants.MIN_ANGLE, HoodConstants.MAX_ANGLE);

        hoodMotor.setControl(request.withPosition(targetAngle));
    }

    /**
     * Gets the current angle of the hood
     * @return
     * current angle
     */
    public Angle getAngle() {
        return hoodMotor.getPosition().getValue();
    }

    /**
     * Gets the target angle of the hood
     * @return
     * targetAngle
     */
    public Angle getTargetAngle() {
        return targetAngle;
    }

    /**
     * Returns true if the hood is on target
     * @return
     * True if on target, false otherwise
     */
    public boolean isOnTarget() {
        return getAngle().isNear(getTargetAngle(), HoodConstants.POSITION_TOLERANCE);
    }

    /**
     * Stops all movement to the hood motor
     */
    public void stop() {
        hoodMotor.stopMotor();
    }

    @Override
    public void simulationPeriodic() {
        double batteryVoltage = RobotController.getBatteryVoltage();
        motorSim.setSupplyVoltage(batteryVoltage);
        encoderSim.setSupplyVoltage(batteryVoltage);

        hoodSim.setInputVoltage(motorSim.getMotorVoltage());
        hoodSim.update(Robot.kDefaultPeriod);

        Angle simAngle = Degrees.of(hoodSim.getAngularPositionRad());
        AngularVelocity simVeloc = DegreesPerSecond.of(hoodSim.getAngularVelocityRadPerSec());

        motorSim.setRawRotorPosition(simAngle);
        motorSim.setRotorVelocity(simVeloc);

        ligament.setAngle(-(simAngle.in(Degrees)) + 270);
        encoderSim.setRawPosition(simAngle);
        encoderSim.setVelocity(simVeloc);

        LightningShuffleboard.setDouble("Hood", "CANcoder angle", encoder.getAbsolutePosition().getValue().in(Degree));
        LightningShuffleboard.setDouble("Hood", "Sim Angle", simAngle.in(Degrees));
        LightningShuffleboard.setDouble("Hood", "Target Angle", getTargetAngle().in(Degrees));
    }
}
