// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Target;
import frc.robot.constants.RobotMap;
import frc.util.hardware.ThunderBird;
import frc.util.shuffleboard.LightningShuffleboard;
import frc.util.units.ThunderUnits;

public class Turret extends SubsystemBase {

    public class TurretConstants {
        public static final boolean INVERTED = true; // temp
        public static final Current STATOR_LIMIT = Amps.of(40); // temp
        public static final boolean BRAKE = false; // temp

        public static final Angle ANGLE_TOLERANCE = Degrees.of(5);

        public static final Angle MIN_ANGLE = Degrees.of(-120); // limit range temporarily
        public static final Angle MAX_ANGLE = Degrees.of(120);

        public static final double kP = 150d;
        public static final double kI = 0d;
        public static final double kD = 9d;
        public static final double kS = 0.33d;

        public static final double ENCODER_TO_MECHANISM_RATIO = 92d / 12d * 5;

        public static final Angle ZERO_ANGLE = Degrees.of(-1.2);
        public static final double ZEROING_POWER = 0.5;

        public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.02);

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
    private boolean lsTriggeredOnLastLoopRun;

    private final Swerve drivetrain;

    private DoubleLogEntry targetPositionLog;
    private BooleanLogEntry onTargetLog;
    private BooleanLogEntry zeroLimitSwitchLog;
    private BooleanLogEntry maxLimitSwitchLog;

    /**
     * Creates a new Turret Subsystem.
     *
     * @param drivetrain the drivetrain to get the pose for the turret sim in
     * advantage scope
     */
    public Turret(Swerve drivetrain) {
        this.drivetrain = drivetrain;

        motor = new ThunderBird(RobotMap.TURRET, RobotMap.CAN_BUS, TurretConstants.INVERTED,
                TurretConstants.STATOR_LIMIT, TurretConstants.BRAKE);

        TalonFXConfiguration config = motor.getConfig();

        config.Slot0.kP = TurretConstants.kP;
        config.Slot0.kI = TurretConstants.kI;
        config.Slot0.kD = TurretConstants.kD;
        config.Slot0.kS = TurretConstants.kS;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TurretConstants.MAX_ANGLE.in(Rotations);
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TurretConstants.MIN_ANGLE.in(Rotations);

        config.Feedback.SensorToMechanismRatio = TurretConstants.ENCODER_TO_MECHANISM_RATIO;

        motor.applyConfig(config);

        zeroLimitSwitch = new DigitalInput(RobotMap.TURRET_ZERO_SWITCH);
        maxLimitSwitch = new DigitalInput(RobotMap.TURRET_MAX_SWITCH);

        zeroed = RobotMap.IS_OASIS || Robot.isSimulation(); // only zero when real // TODO: remove isOasis check after limit switches added
        if (!zeroed) {
            setPower(TurretConstants.ZEROING_POWER); // go toward max switch to zero
        }

        if (Robot.isSimulation()) {
            gearbox = DCMotor.getKrakenX44Foc(1);
            turretSim = new SingleJointedArmSim(gearbox, TurretConstants.ENCODER_TO_MECHANISM_RATIO,
                    TurretConstants.MOI.in(KilogramSquareMeters), TurretConstants.LENGTH.in(Meters),
                    TurretConstants.MIN_ANGLE.in(Radians), TurretConstants.MAX_ANGLE.in(Radians),
                    false, 0);

            motorSim = new TalonFXSimState(motor);

            motorSim.setRawRotorPosition(Degrees.zero());

            mech2d = new Mechanism2d(3, 3);
            root2d = mech2d.getRoot("Turret", 1.5, 1.5);
            ligament = root2d.append(new MechanismLigament2d("Turret", 2, 90));

            LightningShuffleboard.send("Turret", "mech 2d", mech2d);
        }

        initLogging();
    }

    private void initLogging() {
        DataLog log = DataLogManager.getLog();

        targetPositionLog = new DoubleLogEntry(log, "Turret/TargetPosition");
        onTargetLog = new BooleanLogEntry(log, "Turret/OnTarget");
        zeroLimitSwitchLog = new BooleanLogEntry(log, "Turret/ZeroLimitSwitch");
        maxLimitSwitchLog = new BooleanLogEntry(log, "Turret/MaxLimitSwitch");
    }

    @Override
    public void periodic() {
        // Max limit switch will be imprecise, so go the other direction toward zero switch when max is hit
        // if (getMaxLimitSwitch() && !zeroed) {
        //     setPower(-TurretConstants.ZEROING_POWER);
        // }

        // Zero limit switch is precise, so set encoder position when zero is hit
        if (!zeroed && !lsTriggeredOnLastLoopRun && getZeroLimitSwitch() 
            && motor.getVelocity().getValue().lt(RotationsPerSecond.zero())) { // TODO: add led strip indication?

            setEncoderPosition(TurretConstants.ZERO_ANGLE);
            zeroed = true;
            setAngle(targetPosition);
        }

        lsTriggeredOnLastLoopRun = getZeroLimitSwitch();

        updateLogging();
    }

    private void updateLogging() {
        targetPositionLog.append(getTargetAngle().in(Degrees));
        onTargetLog.append(isOnTarget());
        zeroLimitSwitchLog.append(getZeroLimitSwitch());
        maxLimitSwitchLog.append(getMaxLimitSwitch());

        if (!DriverStation.isFMSAttached() || Robot.isSimulation()) {
            LightningShuffleboard.setDouble("Turret", "Current Angle", getAngle().in(Degrees));
            LightningShuffleboard.setDouble("Turret", "Target Angle", getTargetAngle().in(Degrees));
            LightningShuffleboard.setBool("Turret", "On Target", isOnTarget());
            LightningShuffleboard.setBool("Turret", "Zero Limit Switch", getZeroLimitSwitch());
            LightningShuffleboard.setBool("Turret", "Max Limit Switch", getMaxLimitSwitch());
            LightningShuffleboard.setBool("Turret", "Zeroed", zeroed);
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
        motorSim.setRawRotorPosition(simAngle.times(TurretConstants.ENCODER_TO_MECHANISM_RATIO));
        motorSim.setRotorVelocity(simVeloc.times(TurretConstants.ENCODER_TO_MECHANISM_RATIO));

        ligament.setAngle(simAngle.in(Degrees));

        Translation3d robotTranslation3d = new Translation3d(
                drivetrain.getPose().getX(),
                drivetrain.getPose().getY(),
                0.0);

        turretPose3d = new Pose3d(
                robotTranslation3d,
                new Rotation3d(0, 0, getAngle().in(Radians))
        );

        LightningShuffleboard.setPose3d("Turret", "Turret pose 3d", turretPose3d);
    }

    /**
     * sets angle of the turret
     *
     * @param angle sets the angle to the motor of the turret
     */
    public void setAngle(Angle angle) {
        Angle wrappedPosition = ThunderUnits.inputModulus(angle, Degrees.of(-180), Degrees.of(180));

        targetPosition = ThunderUnits.clamp(wrappedPosition, TurretConstants.MIN_ANGLE, TurretConstants.MAX_ANGLE);
        if (zeroed) { // only allow position control if turret has been zeroed but store to apply when zeroed
            motor.setControl(positionPID.withPosition(optimizeTurretAngle(targetPosition)));
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
        return getTargetAngle().isNear(getAngle(), TurretConstants.ANGLE_TOLERANCE) && zeroed;
    }

    /**
     * Limit Switch at zero position
     *
     * @return if zero limit switch triggered
     */
    public boolean getZeroLimitSwitch() {
        return !zeroLimitSwitch.get();
    }

    /**
     * On E-Chain Only really tells us that we are near min or max
     *
     * @return if max limit switch triggered
     */
    public boolean getMaxLimitSwitch() {
        return maxLimitSwitch.get();
    }

    /**
     * Sets the encoder position to a specific angle WANRING: This does not move
     * the turret, only sets the encoder position
     *
     * @param angle
     */
    public void setEncoderPosition(Angle angle) {
        motor.setPosition(angle);
    }

    /**
     * stops all movement to the turret motor
     */
    public void stop() {
        motor.stopMotor();
    }

    /**
     * Extend angle past 180 / -180 if possible while remaining within -220 /
     * 220
     *
     * @param desired the angle between -180 and 180 calculated for the turret
     * @return the angle optimized between -220 and 220
     */
    public Angle optimizeTurretAngle(Angle desired) {
        Angle error = desired.minus(getAngle());
        if (error.in(Degrees) > 180) {
            desired = desired.minus(Degrees.of(360));
        } else if (error.in(Degrees) < -180) {
            desired = desired.plus(Degrees.of(360));
        }

        double minDeg = TurretConstants.MIN_ANGLE.in(Degrees);
        double maxDeg = TurretConstants.MAX_ANGLE.in(Degrees);

        while (desired.lt(Degrees.of(minDeg))) {
            desired = desired.plus(Degrees.of(360));
        }
        while (desired.gt(Degrees.of(maxDeg))) {
            desired = desired.minus(Degrees.of(360));
        }

        return desired;
    }

    public Command turretAim(Target target) {
        return run(() -> {
            Pose2d robotPose = drivetrain.getPose();

            Translation2d delta = FieldConstants.getTargetData(target).minus(robotPose.getTranslation());

            Angle fieldAngle = delta.getAngle().getMeasure();

            Angle turretAngle = fieldAngle.minus(robotPose.getRotation().getMeasure());

            setAngle(turretAngle);
        });
    }

    public Command turretAim(Cannon cannon) {
        return run(() -> {
            Pose2d robotPose = drivetrain.getPose();

            Translation2d target = cannon.getTarget();

            Translation2d delta = target.minus(robotPose.getTranslation());

            Angle fieldAngle = delta.getAngle().getMeasure();

            Angle turretAngle
                    = fieldAngle.minus(robotPose.getRotation().getMeasure());

            setAngle(turretAngle);
        });
    }

    /**
     * Returns a command to set the angle of the turret
     *
     * @param angle The angle to set
     * @return The command
     */
    public Command setAngleCommand(Angle angle) {
        return new InstantCommand(() -> setAngle(angle));
    }

    public boolean getZeroed() {
        return zeroed;
    }
}
