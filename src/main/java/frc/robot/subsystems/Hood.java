// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import java.util.Map;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.Units;
import frc.util.hardware.ThunderBird;
import frc.util.shuffleboard.LightningShuffleboard;
import frc.robot.constants.RobotMap;
import frc.robot.Robot;

public class Hood extends SubsystemBase {

    public class HoodConstants {
        public static final boolean INVERTED = true; // temp
        public static final double STATOR_LIMIT = 40d; // temp
        public static final boolean BRAKE = true; // temp

        public static final Angle MIN_ANGLE = Degree.of(0); // Hood v2
        public static final Angle MAX_ANGLE = Degree.of(30); // Hood v2

        public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.1); // Temp

        // Input is distance to target in meters, output is hood angle in degrees
        public static final InterpolatingDoubleTreeMap HOOD_MAP = InterpolatingDoubleTreeMap.ofEntries(
            Map.entry(2d, 10d),
            Map.entry(4d, 20d),
            Map.entry(6d, 30d));

        public static final double kS = 0.25d; // temp
        public static final double kV = 2; // temp
        public static final double kA = 0.01; // temp
        public static final double kP = 20; // temp
        public static final double kI = 0.0; // temp
        public static final double kD = 0.0; // temp

        public static final Angle POSITION_TOLERANCE = Degree.of(1); // temp

        // Conversion ratios
        public static final double ROTOR_TO_MECHANISM_RATIO = 23.63; // Hood v2
        public static final double ROTOR_TO_ENCODER_RATIO = 1d; // Cancoder mounted on motor
        public static final double ENCODER_TO_MECHANISM_RATIO = ROTOR_TO_MECHANISM_RATIO / ROTOR_TO_ENCODER_RATIO;
    }

    private ThunderBird hoodMotor;
    private CANcoder encoder;

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

        request = new PositionVoltage(0d);

        targetAngle = Degrees.of(0);

        encoder.getConfigurator().apply(angleConfig);

        motorConfig.Slot0.kP = HoodConstants.kP;
        motorConfig.Slot0.kI = HoodConstants.kI;
        motorConfig.Slot0.kD = HoodConstants.kD;
        motorConfig.Slot0.kS = HoodConstants.kS;
        motorConfig.Slot0.kV = HoodConstants.kV;
        motorConfig.Slot0.kA = HoodConstants.kA;

        motorConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        motorConfig.Feedback.SensorToMechanismRatio = HoodConstants.ENCODER_TO_MECHANISM_RATIO;
        motorConfig.Feedback.RotorToSensorRatio = HoodConstants.ROTOR_TO_ENCODER_RATIO;
        hoodMotor.applyConfig(motorConfig);

        if (Robot.isSimulation()) {
            gearbox = DCMotor.getKrakenX44Foc(1);
            hoodSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(gearbox, HoodConstants.MOI.magnitude(), HoodConstants.ROTOR_TO_MECHANISM_RATIO),
                gearbox
            );

            motorSim = new TalonFXSimState(hoodMotor);
            encoderSim = new CANcoderSimState(encoder);

            motorSim.setRawRotorPosition(HoodConstants.MIN_ANGLE.in(Rotations));
            encoderSim.setRawPosition(HoodConstants.MIN_ANGLE.in(Rotations));

            mech2d = new Mechanism2d(2,  2);
            root2d =  mech2d.getRoot("Hood", 0.2, 0.2);

            ligament = root2d.append(new MechanismLigament2d("Hood", 1.5, 0));
            LightningShuffleboard.send("Hood", "Mech2d", mech2d);
        }
    }

    @Override
    public void periodic() {}

    @Override
    public void simulationPeriodic() {
        double batteryVoltage = RobotController.getBatteryVoltage();
        motorSim.setSupplyVoltage(batteryVoltage);
        encoderSim.setSupplyVoltage(batteryVoltage);

        hoodSim.setInputVoltage(motorSim.getMotorVoltage());
        hoodSim.update(Robot.kDefaultPeriod);

        Angle simAngle = Radians.of(hoodSim.getAngularPositionRad());
        AngularVelocity simVeloc = RadiansPerSecond.of(hoodSim.getAngularVelocityRadPerSec());

        motorSim.setRawRotorPosition(simAngle.times(HoodConstants.ROTOR_TO_MECHANISM_RATIO));
        motorSim.setRotorVelocity(simVeloc.times(HoodConstants.ROTOR_TO_MECHANISM_RATIO));

        ligament.setAngle(simAngle.in(Degrees));
        encoderSim.setRawPosition(simAngle.times(HoodConstants.ROTOR_TO_MECHANISM_RATIO));
        encoderSim.setVelocity(simVeloc.times(HoodConstants.ROTOR_TO_MECHANISM_RATIO));

        LightningShuffleboard.setDouble("Hood", "CANcoder angle", encoder.getAbsolutePosition().getValue().in(Degree));
        LightningShuffleboard.setDouble("Hood", "Sim Angle", simAngle.in(Degrees));
        LightningShuffleboard.setDouble("Hood", "Target Angle", getTargetAngle().in(Degrees));
    }

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

    /**
     * angle control command for hood
     * @param hoodAngle
     * @return the command for running the hood
     */
    public Command hoodCommand(Angle hoodAngle) {
        return hoodCommand(() -> hoodAngle);
    }

    /**
     * angle control command for hood
     * @param hoodAngleSupplier
     * @return the command for running the hood
     */
    public Command hoodCommand(Supplier<Angle> hoodAngleSupplier) {
        return new StartEndCommand(() -> setPosition(hoodAngleSupplier.get()), () -> {}, this).until(this::isOnTarget);
    }
}
