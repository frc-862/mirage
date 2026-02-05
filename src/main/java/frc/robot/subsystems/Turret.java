// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.RobotMap;
import frc.util.hardware.ThunderBird;
import frc.util.shuffleboard.LightningShuffleboard;

public class Turret extends SubsystemBase {

    public class TurretConstants {
        public static final boolean INVERTED = false; // temp
        public static final Current STATOR_LIMIT = Amps.of(40); // temp
        public static final boolean BRAKE = false; // temp

        public static final Angle ANGLE_TOLERANCE = Degrees.of(5);

        public static final Angle MIN_ANGLE = Degree.of(-220);
        public static final Angle MAX_ANGLE = Degree.of(220);

        public static final double MOTOR_KP = 6.5;
        public static final double MOTOR_KI = 0;
        public static final double MOTOR_KD = 0;
        public static final double MOTOR_KF = 0;
        public static final double MOTOR_KS = 1;
        public static final double MOTOR_KV = 0.18;
        public static final double MOTOR_KA = 0.01;
        public static final double MOTOR_KG = 0;

        public static final double ENCODER_TO_MECHANISM_RATIO = 74d;

        public static final Angle ZERO_ANGLE = Degree.of(0);
        public static final double ZEROING_POWER = 0.5;

        public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.086);
        public static final Distance LENGTH = Meter.of(0.18);
    }

    private final ThunderBird motor;

    private Angle targetPosition = Rotations.zero();

    public final PositionVoltage positionPID = new PositionVoltage(0);
    private final DutyCycleOut dutyCycle = new DutyCycleOut(0.0);

    private DCMotor gearbox;
    private SingleJointedArmSim turretSim;
    private TalonFXSimState motorSim;

    private Mechanism2d mech2d;
    private MechanismRoot2d root2d;
    private MechanismLigament2d ligament;

    private Pose3d turretPose3d;

    private final DigitalInput zeroLimitSwitch;
    private final DigitalInput maxLimitSwitch;
    private boolean zeroed;

    private final Swerve drivetrain;

    /**
     * Creates a new Turret Subsystem.
     * @param drivetrain the drivetrain to get the pose for the turret sim in advantage scope
     */
    public Turret(Swerve drivetrain) {
        this.drivetrain = drivetrain;

        motor = new ThunderBird(RobotMap.TURRET, RobotMap.CAN_BUS, TurretConstants.INVERTED,
                TurretConstants.STATOR_LIMIT, TurretConstants.BRAKE);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        motorConfig.Slot0.kP = TurretConstants.MOTOR_KP;
        motorConfig.Slot0.kI = TurretConstants.MOTOR_KI;
        motorConfig.Slot0.kD = TurretConstants.MOTOR_KD;
        motorConfig.Slot0.kS = TurretConstants.MOTOR_KS;
        motorConfig.Slot0.kV = TurretConstants.MOTOR_KV;
        motorConfig.Slot0.kA = TurretConstants.MOTOR_KA;
        motorConfig.Slot0.kG = TurretConstants.MOTOR_KG;

        motorConfig.Feedback.SensorToMechanismRatio = TurretConstants.ENCODER_TO_MECHANISM_RATIO;

        motor.applyConfig(motorConfig);

        zeroLimitSwitch = new DigitalInput(RobotMap.TURRET_ZERO_SWITCH);
        maxLimitSwitch = new DigitalInput(RobotMap.TURRET_MAX_SWITCH);

        zeroed = Robot.isSimulation(); // only zero when real
        if (!zeroed) {
            setPower(TurretConstants.ZEROING_POWER); // go toward max switch to zero
        }

        if (Robot.isSimulation()) {
            gearbox = DCMotor.getKrakenX44Foc(1);
            turretSim = new SingleJointedArmSim(gearbox, TurretConstants.ENCODER_TO_MECHANISM_RATIO,
                    TurretConstants.MOI.magnitude(), TurretConstants.LENGTH.in(Meters),
                    TurretConstants.MIN_ANGLE.in(Radians), TurretConstants.MAX_ANGLE.in(Radians),
                    false, TurretConstants.MIN_ANGLE.in(Radians), 0d, 1d);

            motorSim = new TalonFXSimState(motor);

            motorSim.setRawRotorPosition(TurretConstants.MIN_ANGLE.in(Rotations));

            mech2d = new Mechanism2d(3, 3);
            root2d = mech2d.getRoot("Turret", 2, 0);
            ligament = root2d.append(new MechanismLigament2d("Turret", 3, 90));

            LightningShuffleboard.send("Turret", "mech 2d", mech2d);
        }
    }

    @Override
    public void periodic() {

        // Max limit switch will be imprecise, so go the other direction toward zero switch when max is hit
        if (getMaxLimitSwitch() && !zeroed) {
            setPower(-TurretConstants.ZEROING_POWER);
        }

        // Zero limit switch is precise, so stop and set encoder position when zero is hit
        if (getZeroLimitSwitch() && !zeroed) { // TODO: add led strip indication?
            stop();
            setEncoderPosition(TurretConstants.ZERO_ANGLE);
            setAngle(targetPosition);
            zeroed = true;
        }
    }

    @Override
    public void simulationPeriodic() {
        double batteryVoltage = RobotController.getBatteryVoltage();
        motorSim.setSupplyVoltage(batteryVoltage);

        turretSim.setInputVoltage(motorSim.getMotorVoltage());
        turretSim.update(Robot.kDefaultPeriod);

        Angle simAngle = Radians.of(turretSim.getAngleRads());
        AngularVelocity simVeloc = RadiansPerSecond.of(turretSim.getVelocityRadPerSec());
        motorSim.setRawRotorPosition(simAngle);
        motorSim.setRotorVelocity(simVeloc);

        ligament.setAngle(simAngle.in(Degree));

        LightningShuffleboard.setDouble("Turret", "Motor encoder angle",
                motor.getPosition().getValue().in(Degree));
        LightningShuffleboard.setDouble("Turret", "sim angle", simAngle.in(Degree));

        LightningShuffleboard.setDouble("Turret", "current angle", getAngle().in(Degree));
        LightningShuffleboard.setDouble("Turret", "target angle", getTargetAngle().in(Degree));
        LightningShuffleboard.setBool("Turret", "on target", isOnTarget());

        Translation3d robotTranslation3d = new Translation3d(
                drivetrain.getPose().getX(),
                drivetrain.getPose().getY(),
                0.0);

        turretPose3d = new Pose3d(
                robotTranslation3d,
                new Rotation3d(0, 0, getAngle().in(Radians))
        );

        double[] poseArray = new double[] {
                turretPose3d.getX(),
                turretPose3d.getY(),
                turretPose3d.getRotation().getZ()
        };
        LightningShuffleboard.setDoubleArray("Turret", "Turret pose 3d", poseArray);
    }

    /**
     * sets angle of the turret
     *
     * @param angle sets the angle to the motor of the turret
     */
    public void setAngle(Angle angle) {
        targetPosition = angle;
        if (zeroed) { // only allow position control if turret has been zeroed but store to apply when zeroed
            motor.setControl(positionPID.withPosition(targetPosition));
        }
    }

    /**
     * gets the current angle of the turret
     *
     * @return angle of turret
     */
    public Angle getAngle() {
        return motor.getPosition().getValue();
    }

    /**
     * gets the current target angle of the turret
     *
     * @return target angle of turret
     */
    public Angle getTargetAngle() {
        return targetPosition;
    }

    public void setPower(double power) {
        motor.setControl(dutyCycle.withOutput(power));
    }

    /**
     * gets whether the turret is currently on target with set target angle
     *
     * @return whether turret on target
     */
    public boolean isOnTarget() {
        return getTargetAngle().isNear(getAngle(), TurretConstants.ANGLE_TOLERANCE) && zeroed; // only on target if zeroed
    }

    /**
     * Limit Switch at zero position
     * 
     * @return if zero limit switch triggered
     */
    public boolean getZeroLimitSwitch() {
        return zeroLimitSwitch.get();
    }

    /**
     * On E-Chain
     * Only really tells us that we are near min or max
     * 
     * @return if max limit switch triggered
     */
    public boolean getMaxLimitSwitch() {
        return maxLimitSwitch.get();
    }

    /**
     * Sets the encoder position to a specific angle
     * WANRING: This does not move the turret, only sets the encoder position
     *
     * @param angle
     */
    public void setEncoderPosition(Angle angle){
        motor.setPosition(angle);
    }

    /**
     * stops all movement to the turret motor
     */
    public void stop() {
        motor.stopMotor();
    }
}
