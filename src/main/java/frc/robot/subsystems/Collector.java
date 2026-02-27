package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.controls.DutyCycleOut;
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
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.RobotMap;
import frc.util.hardware.ThunderBird;
import frc.util.shuffleboard.LightningShuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
public class Collector extends SubsystemBase {

    public class CollectorConstants {
        // Collector Rollers
        public static final boolean INVERTED = true; // temp
        public static final Current STATOR_LIMIT = Amps.of(80); // temp
        public static final Current CURRENT_THRESHOLD = Amps.of(20); // temp
        public static final boolean BRAKE = true; // temp
        public static final double COLLECT_POWER = 1d;

        public static final MomentOfInertia COLLECTOR_MOI = KilogramSquareMeters.of(0.001); //temp 
        public static final double COLLECTOR_GEAR_RATIO = 1d; //temp

        // pivot
        public static final boolean PIVOT_INVERTED = true; // temp
        public static final Current PIVOT_STATOR_LIMIT = Amps.of(40); // temp
        public static final boolean PIVOT_BRAKE_MODE = true; // temp
        public static final double ROTOR_TO_ENCODER_RATIO = 1d; // temp
        public static final double ENCODER_TO_MECHANISM_RATIO = 36d; // temp
        public static final Angle MIN_ANGLE = Degrees.of(0); // temp
        public static final Angle MAX_ANGLE = Degrees.of(90); // temp
        public static final double DEPLOY_DC = 0.4;
        public static final double DEPLOY_HOLD_DC = 0.05;
        public static final double STOW_DC = -0.4;
        public static final double STOW_HOLD_DC = -0.05;

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
    private final DutyCycleOut pivotDutyCycle;
    private Mechanism2d mech2d;
    private MechanismRoot2d root2d;
    private MechanismLigament2d ligament;
    public enum PIVOT_STATES {
        DISABLED,
        DEPLOYING,
        DEPLOYED,
        STOWING,
        STOWED
    }
    private PIVOT_STATES pivotState = PIVOT_STATES.DISABLED;
    private PIVOT_STATES nextPivotState = PIVOT_STATES.DISABLED;

    private DCMotor gearbox;

    /**
     * Creates a new Collector Subsystem.
     */
    public Collector() {
        collectorMotor = new ThunderBird(RobotMap.COLLECTOR, RobotMap.CAN_BUS,
            CollectorConstants.INVERTED, CollectorConstants.STATOR_LIMIT, CollectorConstants.BRAKE);

        pivotMotor = new ThunderBird(RobotMap.COLLECTOR_PIVOT, RobotMap.CAN_BUS,
            CollectorConstants.PIVOT_INVERTED, CollectorConstants.PIVOT_STATOR_LIMIT, CollectorConstants.PIVOT_BRAKE_MODE);

        collectorDutyCycle = new DutyCycleOut(0.0);
        pivotDutyCycle = new DutyCycleOut(0.0);

        if(Robot.isSimulation()){
            // pivot sim stuff
            gearbox = DCMotor.getKrakenX60Foc(1);

            collectorPivotSim = new SingleJointedArmSim(gearbox, CollectorConstants.ENCODER_TO_MECHANISM_RATIO, CollectorConstants.MOI.magnitude(),
            CollectorConstants.LENGTH.magnitude(), CollectorConstants.MIN_ANGLE.in(Radians), CollectorConstants.MAX_ANGLE.in(Radians), true,
            CollectorConstants.MIN_ANGLE.in(Radians));

            pivotMotorSim = pivotMotor.getSimState();
            pivotMotorSim.Orientation = ChassisReference.Clockwise_Positive;
            pivotMotorSim.setRawRotorPosition(CollectorConstants.MIN_ANGLE);

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
    }

    @Override
    public void periodic() {
        if (pivotState != nextPivotState) {
            switch (nextPivotState) {
                case DEPLOYING:
                    setPivotPower(CollectorConstants.DEPLOY_DC);
                    break;
                case DEPLOYED:
                    setPivotPower(CollectorConstants.DEPLOY_HOLD_DC);
                    break;
                case STOWING:
                    setPivotPower(CollectorConstants.STOW_DC);
                    break;
                case STOWED:
                    setPivotPower(CollectorConstants.STOW_HOLD_DC);
                    break;
                case DISABLED:
                    setPivotPower(0);
                    break;
            }
        }
        switch (pivotState) {
            case DEPLOYING:
                if (pivotMotor.getTorqueCurrent().getValue().gt(CollectorConstants.CURRENT_THRESHOLD)) {
                    nextPivotState = PIVOT_STATES.DEPLOYED;
                }
                break;
            case STOWING:
                if (pivotMotor.getTorqueCurrent().getValue().lt(CollectorConstants.CURRENT_THRESHOLD.unaryMinus())) {
                    nextPivotState = PIVOT_STATES.STOWED;
                }
                break;
            default:
        }
        pivotState = nextPivotState;

        updateLogging();
    }

    private void updateLogging() {

        if (!DriverStation.isFMSAttached() || Robot.isSimulation()) {
            LightningShuffleboard.setString("Collector", "Pivot State", pivotState.toString());
            LightningShuffleboard.setDouble("Collector", "Collector Velocity", getCollectorVelocity().in(RotationsPerSecond));
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

    public PIVOT_STATES getPivotState() {
        return pivotState;
    }

    /**
     * Set the power of the collector motor using duty cycle out
     * @param power duty cycle value from -1.0 to 1.0
     */
    public void setCollectorPower(double power) {
        collectorMotor.setControl(collectorDutyCycle.withOutput(power));
    }

    /**
     * Set the power of the pivot motor using duty cycle out
     * @param power duty cycle value from -1.0 to 1.0
     */
    private void setPivotPower(double power) {
        pivotMotor.setControl(pivotDutyCycle.withOutput(power));
    }

    /**
     * Starts deploying the pivot motor
     */
    public void deployPivot() {
        if (pivotState != PIVOT_STATES.DEPLOYED) {
            nextPivotState = PIVOT_STATES.DEPLOYING;
        }
    }

    /**
     * Starts stowing the pivot motor
     */
    public void stowPivot() {
        if (pivotState != PIVOT_STATES.STOWED) {
            nextPivotState = PIVOT_STATES.STOWING;
        }
    }

    /**
     * Stops the pivot motor
     */
    public void disablePivot() {
        nextPivotState = PIVOT_STATES.DISABLED;
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

    public Command deployPivotCommand() {
        return runOnce(() -> deployPivot());
    }

    public Command stowPivotCommand() {
        return runOnce(() -> stowPivot());
    }

    public Command disablePivotCommand() {
        return runOnce(() -> disablePivot());
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