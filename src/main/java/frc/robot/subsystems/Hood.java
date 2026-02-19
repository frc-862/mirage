// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.RobotMap;
import frc.util.hardware.ThunderBird;
import frc.util.shuffleboard.LightningShuffleboard;
import frc.util.units.ThunderMap;
import frc.util.units.ThunderUnits;

public class Hood extends SubsystemBase {

    public class HoodConstants {
        public static final boolean INVERTED = true; // temp
        public static final Current STATOR_LIMIT = Amps.of(40); // temp
        public static final boolean BRAKE = true; // temp

        public static final Angle MIN_ANGLE = Degrees.of(50); 
        public static final Angle MAX_ANGLE = Degrees.of(80); 

        public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.1); // Temp

        public static final ThunderMap<Distance, Angle> HOOD_MAP = new ThunderMap<>() {
            {
                put(Meters.of(2d), Degrees.of(80));
                put(Meters.of(4d), Degrees.of(60));
                put(Meters.of(8d), Degrees.of(50));
            }
        };

        public static final double kS = 0.05d;
        public static final double kG = -0.3d; // negative because negative power is up
        public static final double kP = 50d;
        public static final double kI = 0.0;
        public static final double kD = 1d;

        public static final Angle POSITION_TOLERANCE = Degrees.of(3); // temp
        public static final Angle BIAS_DELTA = Degrees.of(0.5); // temp

        // Conversion ratios
        public static final double ROTOR_TO_ENCODER_RATIO = !hasEncoder() ? 1 : 50/22d;
        public static final double ENCODER_TO_MECHANISM_RATIO = !hasEncoder() ? 50/22d * 156/15d : 156/15d;
        public static final double ROTOR_TO_MECHANISM_RATIO = ROTOR_TO_ENCODER_RATIO * ENCODER_TO_MECHANISM_RATIO; // only used in sim

        public static final Angle OFFSET_TO_MAX = Rotations.of(0d); // temp
        public static final Angle ENCODER_OFFSET = OFFSET_TO_MAX.plus(MAX_ANGLE);
    }

    private ThunderBird motor;
    private CANcoder encoder;

    final PositionVoltage request;
    private Angle targetAngle;
    private MutAngle hoodBias;

    private DCMotorSim hoodSim;
    private TalonFXSimState motorSim;
    private DCMotor gearbox;
    private MechanismLigament2d ligament;
    private MechanismRoot2d root2d;
    private Mechanism2d mech2d;

    /** Creates a new Hood Subsystem. */
    public Hood() {
        motor = new ThunderBird(RobotMap.HOOD, RobotMap.CAN_BUS, HoodConstants.INVERTED, HoodConstants.STATOR_LIMIT,
            HoodConstants.BRAKE);

        // Do not instantiate if Oasis b/c Oasis doesn't have a CANcoder yet
        if (hasEncoder()) {
            encoder = new CANcoder(RobotMap.HOOD_ENCODER, RobotMap.CAN_BUS);
        }

        TalonFXConfiguration motorConfig = motor.getConfig();

        request = new PositionVoltage(0d);

        targetAngle = Degrees.zero();

        hoodBias = Degrees.mutable(0);

        if (hasEncoder()) {
            CANcoderConfiguration angleConfig = new CANcoderConfiguration();
            angleConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5d;
            angleConfig.MagnetSensor.MagnetOffset = Robot.isReal() ? HoodConstants.ENCODER_OFFSET.in(Rotations) : 0d;
            angleConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
            encoder.getConfigurator().apply(angleConfig);
        }

        motorConfig.Slot0.kP = HoodConstants.kP;
        motorConfig.Slot0.kI = HoodConstants.kI;
        motorConfig.Slot0.kD = HoodConstants.kD;
        motorConfig.Slot0.kS = HoodConstants.kS;
        motorConfig.Slot0.kG = HoodConstants.kG;

        if (hasEncoder()) {
            motorConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
            motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        }

        motorConfig.Feedback.SensorToMechanismRatio = HoodConstants.ENCODER_TO_MECHANISM_RATIO;
        motorConfig.Feedback.RotorToSensorRatio = HoodConstants.ROTOR_TO_ENCODER_RATIO;

        motor.applyConfig(motorConfig);



        if (Robot.isSimulation()) {
            gearbox = DCMotor.getKrakenX44Foc(1);
            hoodSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(gearbox, HoodConstants.MOI.magnitude(), HoodConstants.ROTOR_TO_MECHANISM_RATIO),
                gearbox
            );

            motorSim = motor.getSimState();
            motorSim.Orientation = ChassisReference.Clockwise_Positive;

            hoodSim.setState(HoodConstants.MAX_ANGLE.in(Radians), 0);
            motorSim.setRawRotorPosition(HoodConstants.MAX_ANGLE.times(HoodConstants.ROTOR_TO_MECHANISM_RATIO));
           
            mech2d = new Mechanism2d(2,  2);
            root2d =  mech2d.getRoot("Hood", 0.2, 0.2);

            ligament = root2d.append(new MechanismLigament2d("Hood", 1.5, HoodConstants.MAX_ANGLE.in(Degrees)));
            LightningShuffleboard.send("Hood", "Mech2d", mech2d);
        }
        if (!hasEncoder()){
            motor.setPosition(HoodConstants.MAX_ANGLE); // needs to be after config and sim
        }
    }

    private static boolean hasEncoder(){
        return !RobotMap.IS_OASIS && !Robot.isSimulation();
    }

    @Override
    public void periodic() {
        LightningShuffleboard.setDouble("Hood", "Angle (Degrees)", getAngle().in(Degrees));
    }

    @Override
    public void simulationPeriodic() {
        double batteryVoltage = RobotController.getBatteryVoltage();
        motorSim.setSupplyVoltage(batteryVoltage);

        hoodSim.setInputVoltage(motorSim.getMotorVoltage());
        hoodSim.update(Robot.kDefaultPeriod);

        Angle simAngle = Radians.of(hoodSim.getAngularPositionRad());
        AngularVelocity simVeloc = RadiansPerSecond.of(hoodSim.getAngularVelocityRadPerSec());

        motorSim.setRawRotorPosition(simAngle.times(HoodConstants.ROTOR_TO_MECHANISM_RATIO));
        motorSim.setRotorVelocity(simVeloc.times(HoodConstants.ROTOR_TO_MECHANISM_RATIO));

        ligament.setAngle(simAngle.in(Degrees));

        LightningShuffleboard.setDouble("Hood", "Sim Angle", simAngle.in(Degrees));
        LightningShuffleboard.setDouble("Hood", "Target Angle", getTargetAngle().in(Degrees));
        LightningShuffleboard.setDouble("Hood", "Bias", getBias().in(Degrees));
        LightningShuffleboard.setBool("Hood", "On Target", isOnTarget());
    }

    /**
     * Sets position of the hood
     * @param position in degrees
     */
    public void setPosition(Angle position) {
        targetAngle = ThunderUnits.clamp(position, HoodConstants.MIN_ANGLE, HoodConstants.MAX_ANGLE);
        applyControl();
    }

    /**
     * Changes the bias and adds hoodBias to bias. Adds a certain amount of degrees to the hood's target position.
     * @param bias the amount of degrees to add to the hood target position going forward.
     */
    public void changeBias(Angle bias) {
        hoodBias.mut_plus(bias);
        applyControl();
    }

    public Command changeBiasCommand(Angle bias) {
        return new InstantCommand(() -> changeBias(bias));
    }

    public void setBias(Angle bias) {
        hoodBias.mut_replace(bias);
        applyControl();
    }

    private void applyControl() {
        motor.setControl(request.withPosition(getTargetAngleWithBias()));
    }

    
    /**
     * Gets the current angle of the hood
     * @return current angle
     */
    public Angle getAngle() {
        return motor.getPosition().getValue();
    }

    /**
     * Gets the target angle of the hood
     * @return target angle without the bias.
     */
    public Angle getTargetAngle() {
        return targetAngle;
    }

    /**
     * Gets the target angle with bias.
     * @return target angle with the bias added.
     */
    public Angle getTargetAngleWithBias() {
        return ThunderUnits.clamp(targetAngle.plus(hoodBias), HoodConstants.MIN_ANGLE, HoodConstants.MAX_ANGLE);
    }

    /**
     * Gets the bias of the hood
     * @return the bias of hood.
     */
    public Angle getBias() {
        return hoodBias;
    }

    /**
     * Returns true if the hood is on target
     * @return
     * True if on target, false otherwise
     */
    public boolean isOnTarget() {
        return getAngle().isNear(getTargetAngleWithBias(), HoodConstants.POSITION_TOLERANCE);
    }

    /**
     * Stops all movement to the hood motor
     */
    public void stop() {
        motor.stopMotor();
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

    /**
     * keeps the hood pointed at the target of the Robot.
     * @param drivetrain
     * @return Command for repositioning the hood.
     */
    public Command hoodAim(Swerve drivetrain){
        return run(() -> {
            Distance distance = Meters.of(drivetrain.getShooterTranslation().getDistance(drivetrain.getTargetPosition()));
            Angle targetAngle = HoodConstants.HOOD_MAP.get(distance);
            setPosition(targetAngle);
        });
    }

    /**
     * Stows the hood to the max angle for trench
     * Has to be seperate becuase it cannot end automatically
     * @return command
     */
    public Command hoodStowCommand() {
        return startEnd(() -> setPosition(HoodConstants.MAX_ANGLE), this::stop);
    }
}
