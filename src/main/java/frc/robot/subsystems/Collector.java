package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.RobotMap;
import frc.util.hardware.ThunderBird;
import frc.util.shuffleboard.LightningShuffleboard;
import frc.util.units.ThunderUnits;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
public class Collector extends SubsystemBase {

    public class CollectorConstants {
        // Collector Rollers
        public static final boolean INVERTED = false; // temp
        public static final Current STATOR_LIMIT = Amps.of(80); // temp
        public static final Current CURRENT_THRESHOLD = Amps.of(20); // temp
        public static final boolean BRAKE = true; // temp
        public static final double COLLECT_POWER = 1d;

        public static final MomentOfInertia COLLECTOR_MOI = KilogramSquareMeters.of(0.001); //temp 
        public static final double COLLECTOR_GEAR_RATIO = 1d; //temp

        // pivot motor config
        public static final double PIVOT_KP = 15d; // temp
        public static final double PIVOT_KI = 0d; // temp
        public static final double PIVOT_KD = 0d; // temp
        public static final double PIVOT_KS = 0; // temp
        public static final double PIVOT_KG = 0d; // temp


        // pivot
        public static final boolean PIVOT_INVERTED = true; // temp
        public static final Current PIVOT_STATOR_LIMIT = Amps.of(40); // temp
        public static final boolean PIVOT_BRAKE_MODE = true; // temp
        public static final double ROTOR_TO_ENCODER_RATIO = 1d; // temp
        public static final double ENCODER_TO_MECHANISM_RATIO = 9d * 24d/10d; // temp
        public static final Angle MIN_ANGLE = Rotations.of(0);
        public static final Angle MAX_ANGLE = Rotations.of(0.35);
        public static final Angle DEPLOY_ANGLE = MAX_ANGLE;
        public static final Angle STOW_ANGLE = MIN_ANGLE;
        public static final Angle TOLERANCE = Rotations.of(0.05);
        public static final DutyCycleOut PIVOT_ZEROING_DC = new DutyCycleOut(-0.1);

        public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.01); // temp
        public static final Distance LENGTH = Inches.of(6);

        // Sim
        public static final Distance WIDTH = Inches.of(27); // temp
        public static final Distance LENGTH_EXTENDED = Inches.of(7); // temp
        public static final int ROBOT_FUEL_CAPACITY = 50; // temp
        public static final AngularVelocity SIM_COLLECTING_THRESHOLD = RotationsPerSecond.of(1); // temp
    }

    // Collector Rollers
    private ThunderBird collectorMotor;
    private TalonFXSimState collectorMotorSim;
    private DCMotorSim collectorSim;
    private final DutyCycleOut collectorDutyCycle;
    private DCMotor collectorGearbox;

    // Pivot
    private ThunderBird pivotMotor;
    private TalonFXSimState pivotMotorSim;
    private SingleJointedArmSim collectorPivotSim;
    private Mechanism2d mech2d;
    private MechanismRoot2d root2d;
    private MechanismLigament2d ligament;
    private Angle targetPivotPosition;
    private PositionVoltage positionPID;
    private boolean pivotZeroed = true;
    private final Timer zeroingTimer = new Timer();

    private DCMotor gearbox;

    private DoubleLogEntry pivotTargetAngleLog;
    private BooleanLogEntry pivotOnTargetLog;

    /**
     * Creates a new Collector Subsystem.
     */
    public Collector() {
        collectorMotor = new ThunderBird(RobotMap.COLLECTOR, RobotMap.CAN_BUS,
            CollectorConstants.INVERTED, CollectorConstants.STATOR_LIMIT, CollectorConstants.BRAKE);

        pivotMotor = new ThunderBird(RobotMap.COLLECTOR_PIVOT, RobotMap.CAN_BUS,
            CollectorConstants.PIVOT_INVERTED, CollectorConstants.PIVOT_STATOR_LIMIT, CollectorConstants.PIVOT_BRAKE_MODE);

        targetPivotPosition = CollectorConstants.STOW_ANGLE;

        collectorDutyCycle = new DutyCycleOut(0.0);

        TalonFXConfiguration config = pivotMotor.getConfig();
        positionPID = new PositionVoltage(0);

        config.Slot0.kP = CollectorConstants.PIVOT_KP;
        config.Slot0.kI = CollectorConstants.PIVOT_KI;
        config.Slot0.kD = CollectorConstants.PIVOT_KD;
        config.Slot0.kS = CollectorConstants.PIVOT_KS;

        config.Feedback.SensorToMechanismRatio = CollectorConstants.ENCODER_TO_MECHANISM_RATIO;

        pivotMotor.applyConfig(config);

        if(Robot.isSimulation()){
            // pivot sim stuff
            gearbox = DCMotor.getKrakenX60Foc(1);

            collectorPivotSim = new SingleJointedArmSim(gearbox, CollectorConstants.ENCODER_TO_MECHANISM_RATIO, CollectorConstants.MOI.magnitude(),
            CollectorConstants.LENGTH.in(Meters), CollectorConstants.MIN_ANGLE.in(Radians), CollectorConstants.MAX_ANGLE.in(Radians), true,
            CollectorConstants.STOW_ANGLE.in(Radians));

            pivotMotorSim = pivotMotor.getSimState();
            pivotMotorSim.Orientation = ChassisReference.Clockwise_Positive;
            pivotMotorSim.setRawRotorPosition(CollectorConstants.STOW_ANGLE);

            // collector sim stuff
            collectorGearbox = DCMotor.getKrakenX60(1);
            collectorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(collectorGearbox, CollectorConstants.COLLECTOR_MOI.magnitude(), CollectorConstants.COLLECTOR_GEAR_RATIO),
                collectorGearbox
            );

            collectorMotorSim = collectorMotor.getSimState();
            collectorMotorSim.setMotorType(MotorType.KrakenX60);

            mech2d = new Mechanism2d(3, 3);
            root2d = mech2d.getRoot("Collector", 0.5, 0.5);
            ligament = root2d.append(new MechanismLigament2d("Collector", 2, 90));
            
            LightningShuffleboard.send("Collector", "mech 2d", mech2d);
        }

        initLogging();
    }

    private void initLogging() {
        DataLog log = DataLogManager.getLog();

        pivotTargetAngleLog = new DoubleLogEntry(log, "/Collector/pivotTargetAngle");
        pivotOnTargetLog = new BooleanLogEntry(log, "/Collector/pivotOnTarget");

        LightningShuffleboard.setBool("Collector", "Request Zeroing", false);
    }

    @Override
    public void periodic() {
        if (LightningShuffleboard.getBool("Collector", "Request Zeroing", false)) {
            pivotZeroed = false;
        }
        if (!pivotZeroed && DriverStation.isEnabled()) {
            if (!zeroingTimer.isRunning()) {
                zeroingTimer.restart();
                pivotMotor.setControl(CollectorConstants.PIVOT_ZEROING_DC);
            } else if (!pivotMotor.getVelocity().getValue().isNear(RotationsPerSecond.zero(), RotationsPerSecond.of(0.1))) {
                zeroingTimer.reset();
            } else if (zeroingTimer.hasElapsed(0.2)) {
                pivotZeroed = true;
                pivotMotor.setPosition(CollectorConstants.STOW_ANGLE);
                zeroingTimer.stop();
                LightningShuffleboard.setBool("Collector", "Request Zeroing", false);
                setPivotAngle(targetPivotPosition);
            }
        }
        updateLogging();
    }

    private void updateLogging() {
        pivotTargetAngleLog.append(getTargetPivotAngle().in(Degrees));
        pivotOnTargetLog.append(pivotOnTarget());

        if (!DriverStation.isFMSAttached() || Robot.isSimulation()) {
            LightningShuffleboard.setDouble("Collector", "Pivot Rotations", getPivotAngle().in(Rotations));
            LightningShuffleboard.setDouble("Collector", "Pivot Target Rotations", getTargetPivotAngle().in(Rotations));
            LightningShuffleboard.setBool("Collector", "Pivot On Target", pivotOnTarget());
            LightningShuffleboard.setDouble("Collector", "Collector Velocity", getCollectorVelocity().in(RotationsPerSecond));
            LightningShuffleboard.setBool("Collector", "Pivot Zeroed", pivotZeroed);
            LightningShuffleboard.setDouble("Collector", "Pivot Timer", zeroingTimer.get());
        }
    }


    @Override
    public void simulationPeriodic() {
        // collector sim stuff
        collectorMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        collectorSim.setInputVoltage(collectorMotorSim.getMotorVoltage());
        collectorSim.update(Robot.kDefaultPeriod);

        Angle collectorSimAngle = Radians.of(collectorSim.getAngularPositionRad());
        AngularVelocity collectorSimVelocity = RadiansPerSecond.of(collectorSim.getAngularVelocityRadPerSec());

        collectorMotorSim.setRawRotorPosition(collectorSimAngle.times(CollectorConstants.COLLECTOR_GEAR_RATIO));
        collectorMotorSim.setRotorVelocity(collectorSimVelocity.times(CollectorConstants.COLLECTOR_GEAR_RATIO));

        // pivot sim stuff
        pivotMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        collectorPivotSim.setInputVoltage(pivotMotorSim.getMotorVoltage());
        collectorPivotSim.update(Robot.kDefaultPeriod);

        Angle pivotSimAngle = Radians.of(collectorPivotSim.getAngleRads());
        AngularVelocity pivotSimVelocity = RadiansPerSecond.of(collectorPivotSim.getVelocityRadPerSec());
        
        pivotMotorSim.setRawRotorPosition(pivotSimAngle.times(CollectorConstants.ENCODER_TO_MECHANISM_RATIO));
        pivotMotorSim.setRotorVelocity(pivotSimVelocity.times(CollectorConstants.ENCODER_TO_MECHANISM_RATIO));
        ligament.setAngle(90 - pivotSimAngle.in(Degrees));
    }

    /**
     * Set the power of the collector motor using duty cycle out
     * @param power duty cycle value from -1.0 to 1.0
     */
    public void setCollectorPower(double power) {
        collectorMotor.setControl(collectorDutyCycle.withOutput(power));
    }

    /**
     * Starts deploying the pivot motor
     */
    public void deployPivot() {
        setPivotAngle(CollectorConstants.DEPLOY_ANGLE);
    }

    /**
     * Starts stowing the pivot motor
     */
    public void stowPivot() {
        setPivotAngle(CollectorConstants.STOW_ANGLE);
    }

    /**
     * Stops all movement to the collector motor
     */
    public void stopCollector() {
        collectorMotor.stopMotor();
    }

    /**
     * Gets the current velocity of the collector motor.
     *
     * @return the collector motor velocity as an {@link AngularVelocity}
     */
    public AngularVelocity getCollectorVelocity() {
        return collectorMotor.getVelocity().getValue();
    }

    /**
     * Set the pivot position in degrees
     * @param position in degrees
     */
    public void setPivotAngle(Angle position) {
        targetPivotPosition = ThunderUnits.clamp(position, CollectorConstants.MIN_ANGLE, CollectorConstants.MAX_ANGLE);
        if (pivotZeroed) {
            pivotMotor.setControl(positionPID.withPosition(targetPivotPosition));
        }
    }


    /**
     * gets the target position of the pivot as an {@link Angle}
     * @return the target position of the pivot
     */
    public Angle getTargetPivotAngle() {
        return targetPivotPosition;
    }

     /**
     * Get the angle of the pivot
     * @return angle of the pivot
     */
    public Angle getPivotAngle(){
        return pivotMotor.getPosition().getValue();
    }

    /**
     * Checks if the wrist is on target
     *
     * @return True if the wrist is on target
     */
    public boolean pivotOnTarget() {
        return targetPivotPosition.isNear(getPivotAngle(), CollectorConstants.TOLERANCE);
    }

    public boolean isDeployed() {
        return pivotOnTarget() && targetPivotPosition.isEquivalent(CollectorConstants.DEPLOY_ANGLE);
    }

    public boolean isStowed() {
        return pivotOnTarget() && targetPivotPosition.isEquivalent(CollectorConstants.STOW_ANGLE);
    }

    public Command deployPivotCommand() {
        return runOnce(() -> deployPivot());
    }

    public Command stowPivotCommand() {
        return runOnce(() -> stowPivot());
    }

    public Command collectCommand(DoubleSupplier power) {
        return new FunctionalCommand(
            () -> deployPivot(),
            () -> setCollectorPower(power.getAsDouble()),
            (interrupted) -> stopCollector(),
            () -> false,
            this
        );
    }
}
