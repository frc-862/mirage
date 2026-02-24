package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

import javax.swing.undo.StateEdit;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
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
        // motor rollers
        public static final boolean INVERTED = true; // temp
        public static final Current STATOR_LIMIT = Amps.of(80); // temp
        public static final Current CURRENT_THRESHOLD = Amps.of(20); // temp
        public static final boolean BRAKE = true; // temp
        public static final double COLLECT_POWER = 1d;
        public static final double DEPLOY_POWER = 0.4d; // temp
        public static final double HOLD_POWER = 0.1d; // temp

        public static final MomentOfInertia COLLECTOR_MOI = KilogramSquareMeters.of(0.001); //temp 
        public static final double COLLECTOR_GEAR_RATIO = 1d; //temp

        // pivot motor config
        public static final double PIVOT_KP = 50d; // temp
        public static final double PIVOT_KI = 0d; // temp
        public static final double PIVOT_KD = 0d; // temp
        public static final double PIVOT_KS = 0; // temp
        public static final double PIVOT_KG = 0d; // temp

        // pivot
        public static final boolean PIVOT_INVERTED = false; // temp
        public static final Current PIVOT_STATOR_LIMIT = Amps.of(40); // temp
        public static final boolean PIVOT_BRAKE_MODE = true; // temp
        public static final double PIVOT_OFFSET = -0.227; // temp
        public static final double ROTOR_TO_ENCODER_RATIO = 1d; // temp
        public static final double ENCODER_TO_MECHANISM_RATIO = 36d; // temp
        public static final Angle MIN_ANGLE = Degrees.of(0); // temp
        public static final Angle MAX_ANGLE = Degrees.of(90); // temp
        public static final Angle DEPLOY_ANGLE = MAX_ANGLE;
        public static final Angle STOWED_ANGLE = MIN_ANGLE;
        public static final Angle TOLERANCE = Degrees.of(2); // temp

        public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.01); // temp
        public static final Distance LENGTH = Inches.of(6);

        // Sim
        public static final Distance WIDTH = Inches.of(27); // temp
        public static final Distance LENGTH_EXTENDED = Inches.of(7); // temp
        public static final int ROBOT_FUEL_CAPACITY = 50; // temp
        public static final AngularVelocity SIM_COLLECTING_THRESHOLD = RotationsPerSecond.of(1); // temp
    }

    //Collector
    private ThunderBird collectorMotor;
    private TalonFXSimState collectorMotorSim;
    private DCMotorSim collectorSim;
    private final DutyCycleOut collectorDutyCycle;
    private DCMotor collectorGearbox;

    //Pivot
    private ThunderBird pivotMotor;
    private TalonFXSimState pivotMotorSim;
    private SingleJointedArmSim collectorPivotSim;
    private Mechanism2d mech2d;
    private MechanismRoot2d root2d;
    private MechanismLigament2d ligament;
    private Angle targetPivotPosition;
    @SuppressWarnings("unused")
    private final PositionVoltage positionPID;

    private DCMotor gearbox;

    /**
     * Creates a new Collector Subsystem.
     */
    public Collector() {
        collectorMotor = new ThunderBird(RobotMap.COLLECTOR, RobotMap.CAN_BUS,
            CollectorConstants.INVERTED, CollectorConstants.STATOR_LIMIT, CollectorConstants.BRAKE);

        pivotMotor = new ThunderBird(RobotMap.COLLECTOR_PIVOT, RobotMap.CAN_BUS,
            CollectorConstants.PIVOT_INVERTED, CollectorConstants.PIVOT_STATOR_LIMIT, CollectorConstants.PIVOT_BRAKE_MODE);

        targetPivotPosition = CollectorConstants.STOWED_ANGLE;

        collectorDutyCycle = new DutyCycleOut(0.0);
        positionPID = new PositionVoltage(0);

        TalonFXConfiguration config = pivotMotor.getConfig();
        config.Slot0.kP = CollectorConstants.PIVOT_KP;
        config.Slot0.kI = CollectorConstants.PIVOT_KI;
        config.Slot0.kD = CollectorConstants.PIVOT_KD;
        config.Slot0.kS = CollectorConstants.PIVOT_KS;
        config.Slot0.kG = CollectorConstants.PIVOT_KG;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        config.Feedback.SensorToMechanismRatio = CollectorConstants.ENCODER_TO_MECHANISM_RATIO;

        pivotMotor.applyConfig(config);

        if(Robot.isSimulation()){
            // pivot sim stuff
            gearbox = DCMotor.getKrakenX60Foc(1);

            collectorPivotSim = new SingleJointedArmSim(gearbox, CollectorConstants.ENCODER_TO_MECHANISM_RATIO, CollectorConstants.MOI.magnitude(),
            CollectorConstants.LENGTH.magnitude(), CollectorConstants.MIN_ANGLE.in(Radians), CollectorConstants.MAX_ANGLE.in(Radians), true,
            CollectorConstants.STOWED_ANGLE.in(Radians));

            pivotMotorSim = pivotMotor.getSimState();
            pivotMotorSim.setRawRotorPosition(CollectorConstants.STOWED_ANGLE);

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
    }

    @Override
    public void periodic() {
        // if (pivotMotor.getStatorCurrent().getValue().gt((CollectorConstants.COLLECTOR_MOTOR_CURRENT_THRESHOLD))) {
        //     setPivotAngle(Degrees.of(90));
        // }
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
        ligament.setAngle(90 - getPivotAngle().in(Degrees));

        LightningShuffleboard.setDouble("Collector", "Collector Pivot Position", getPivotAngle().in(Degrees));
        LightningShuffleboard.setDouble("Collector", "Collector Target Angle", targetPivotPosition.in(Degrees));
        LightningShuffleboard.setBool("Collector", "Collector On Target", pivotOnTarget());
        LightningShuffleboard.setDouble("Collector", "Collector Velocity", collectorSimVelocity.in(RotationsPerSecond));
    }

    /**
     * Set the power of the collector motor using duty cycle out
     * @param power duty cycle value from -1.0 to 1.0
     */
    public void setCollectorPower(double power) {
        collectorMotor.setControl(collectorDutyCycle.withOutput(power));
    }

    /**
     * Deploys the collector with specified power and position
     * @param power applying power with duty cycle
     * @param position set the pivot position
     */
    public void deployCollector(double power, Angle position) {
        setCollectorPower(power);
        setPivotAngle(position);
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
    public AngularVelocity getVelocity() {
        return collectorMotor.getVelocity().getValue();
    }

    /**
     * Set the pivot position in degrees
     *
     * @param position in degrees
     */
    public void setPivotAngle(Angle position) {
        targetPivotPosition = ThunderUnits.clamp(position, CollectorConstants.MIN_ANGLE, CollectorConstants.MAX_ANGLE);
        
        // pivotMotor.setControl(positionPID.withPosition(targetPivotPosition)); // TODO: Uncomment when pivot is figured out
    }

    /**
     * Checks if the wrist is on target
     *
     * @return True if the wrist is on target
     */
    public boolean pivotOnTarget() {
        return targetPivotPosition.isNear(getPivotAngle(), CollectorConstants.TOLERANCE);
    }

    /**
     * dutycycleout command for collector
     * @param power
     * @param position
     * @return the command for running the collector
     */
    public Command collectCommand(double power, Angle position) {
        return new StartEndCommand(() -> deployCollector(power, position), () -> stopCollector(), this);
    }

    public Command collectCommand(double power) {
        return new StartEndCommand(() -> setCollectorPower(power), () -> stopCollector(), this);
    }

    public Command collectRunCommand(DoubleSupplier power) {
        return runEnd(() -> setCollectorPower(power.getAsDouble()), () -> stopCollector());
    }

    public Command pivotCommand(Angle position) {
        return new InstantCommand(() -> setPivotAngle(position)); // Cannot require collector b/c of default command
    }

    /**
     * Get the angle of the pivot
     * @return angle of the pivot
     */
    public Angle getPivotAngle(){
        return pivotMotor.getPosition().getValue();
    }
}
