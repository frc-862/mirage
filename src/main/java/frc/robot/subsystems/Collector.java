package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Robot;
import frc.robot.constants.RobotMap;
import frc.util.hardware.ThunderBird;
import frc.util.shuffleboard.LightningShuffleboard;

public class Collector extends SubsystemBase {

    public class CollectorConstants {
        // motor rollers
        public static final boolean COLLECTOR_MOTOR_INVERTED = false; // temp
        public static final double COLLECTOR_MOTOR_STATOR_LIMIT = 40d; // temp
        public static final boolean COLLECTOR_MOTOR_BRAKE = true; // temp
        public static final double COLLECT_POWER = 1d;

        public static final double COLLECTOR_SIM_kV = 0.24; // temp
        public static final double COLLECTOR_SIM_kA = 0.8; // temp

        // pivot motor config
        public static final double PIVOT_KP = 3d; // temp
        public static final double PIVOT_KI = 0d; // temp
        public static final double PIVOT_KD = 1d; // temp
        public static final double PIVOT_KS = 0; // temp
        public static final double PIVOT_KV = 0; // temp
        public static final double PIVOT_KA = 0; // temp
        public static final double PIVOT_KG = 0d; // temp

        // pivot
        public static final boolean PIVOT_INVERTED = false; // temp
        public static final double PIVOT_STATOR_LIMIT = 40d; // temp
        public static final boolean PIVOT_BRAKE_MODE = true; // temp
        public static final double PIVOT_OFFSET = -0.227; // temp
        public static final double ROTOR_TO_ENCODER_RATIO = 74; // temp
        public static final double ENCODER_TO_MECHANISM_RATIO = 1d; // temp
        public static final Angle MIN_ANGLE = Degrees.of(-85); // temp
        public static final Angle MAX_ANGLE = Degrees.of(85); // temp
        public static final Angle TOLERANCE = Degrees.of(5); // temp

        public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.1); // temp
        public static final Distance LENGTH = Inches.of(6);

        // Sim
        public static final Distance WIDTH = Inches.of(27); // temp
        public static final Distance LENGTH_EXTENDED = Inches.of(7); // temp
        public static final int ROBOT_FUEL_CAPACITY = 50; // temp
        public static final AngularVelocity SIM_COLLECTING_THRESHOLD = RotationsPerSecond.of(1); // temp
    }

    private ThunderBird collectorMotor;
    private ThunderBird pivotMotor;
    private TalonFXSimState pivotSim;
    private SingleJointedArmSim collectorPivotSim;

    private double simMechanismPosition = 0.0; // Track position in rotations
    private TalonFXSimState collectorMotorSim;

    private final DutyCycleOut collectorDutyCycle;

    private Angle targetPivotPosition;
    private final PositionVoltage positionPID;

    private DCMotor gearbox;

    /**
     * Creates a new Collector Subsystem.
     */
    public Collector() {
        collectorMotor = new ThunderBird(RobotMap.COLLECTOR, RobotMap.CAN_BUS,
            CollectorConstants.COLLECTOR_MOTOR_INVERTED, CollectorConstants.COLLECTOR_MOTOR_STATOR_LIMIT, CollectorConstants.COLLECTOR_MOTOR_BRAKE);

        pivotMotor = new ThunderBird(RobotMap.COLLECTOR_PIVOT, RobotMap.CAN_BUS,
            CollectorConstants.PIVOT_INVERTED, CollectorConstants.PIVOT_STATOR_LIMIT, CollectorConstants.PIVOT_BRAKE_MODE);

        targetPivotPosition = Degrees.of(0d);

        collectorDutyCycle = new DutyCycleOut(0.0);
        positionPID = new PositionVoltage(0);

        TalonFXConfiguration config = pivotMotor.getConfig();
        config.Slot0.kP = CollectorConstants.PIVOT_KP;
        config.Slot0.kI = CollectorConstants.PIVOT_KI;
        config.Slot0.kD = CollectorConstants.PIVOT_KD;
        config.Slot0.kS = CollectorConstants.PIVOT_KS;
        config.Slot0.kV = CollectorConstants.PIVOT_KV;
        config.Slot0.kA = CollectorConstants.PIVOT_KA;
        config.Slot0.kG = CollectorConstants.PIVOT_KG;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        config.Feedback.SensorToMechanismRatio = CollectorConstants.ENCODER_TO_MECHANISM_RATIO;
        config.Feedback.RotorToSensorRatio = CollectorConstants.ROTOR_TO_ENCODER_RATIO;

        pivotMotor.applyConfig(config);

        if(Robot.isSimulation()){
            // pivot sim stuff
            gearbox = DCMotor.getKrakenX44Foc(1);

            collectorPivotSim = new SingleJointedArmSim(gearbox, CollectorConstants.ROTOR_TO_ENCODER_RATIO, CollectorConstants.MOI.magnitude(),
            CollectorConstants.LENGTH.magnitude(), CollectorConstants.MIN_ANGLE.in(Radians), CollectorConstants.MAX_ANGLE.in(Radians), false,
            CollectorConstants.MIN_ANGLE.in(Radians), 0d,1d);

            pivotSim = pivotMotor.getSimState();
            pivotSim.setRawRotorPosition(CollectorConstants.MIN_ANGLE.in(Radians));

            // collector sim stuff
            collectorMotorSim = collectorMotor.getSimState();
            collectorMotorSim.setMotorType(MotorType.KrakenX60);
        }
    }

    @Override
    public void periodic() {}

    @Override
    public void simulationPeriodic() {
        // pivot sim stuff
        pivotSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        Angle simAngle = Radians.of(collectorPivotSim.getAngleRads());

        // collector sim stuff
        collectorMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        collectorPivotSim.setInput(collectorMotorSim.getMotorVoltageMeasure().in(Volts));
        collectorPivotSim.update(Robot.kDefaultPeriod);

        collectorMotorSim.setRotorVelocity(collectorPivotSim.getOutput(0));

        pivotSim.setRawRotorPosition(simAngle);

        //Get the motor voltage output
        double motorVoltage = pivotSim.getMotorVoltage();

        //Update the physics simulation with voltage input
        collectorPivotSim.setInput(motorVoltage);
        collectorPivotSim.update(0.020); // 20ms period

        // pivotSim = new SingleJointedArmSim(null, motorVoltage, motorVoltage, motorVoltage, motorVoltage, motorVoltage, false, motorVoltage, null);

        // Get simulated velocity (output of velocity system) in rotations/sec
        double mechanismVelocity = collectorPivotSim.getOutput(0);

        //Integrate velocity to get position
        simMechanismPosition += mechanismVelocity * 0.020; // position in rotations

        // Convert mechanism values to rotor values (multiply by gear ratio)
        double rotorPosition = simMechanismPosition * CollectorConstants.ROTOR_TO_ENCODER_RATIO;
        double rotorVelocity = mechanismVelocity * CollectorConstants.ROTOR_TO_ENCODER_RATIO;

        //Apply the simulated state back to the motor
        pivotSim.setRawRotorPosition(rotorPosition);
        pivotSim.setRotorVelocity(rotorVelocity);

        LightningShuffleboard.setDouble("Collector", "Collector Pivot Position", getPivotAngle().in(Degrees));
        LightningShuffleboard.setDouble("Collector", "target angle", getPivotTargetAngle().in(Degrees));
        LightningShuffleboard.setBool("Collector", "on target?", pivotOnTarget());
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
        pivotMotor.setControl(positionPID.withPosition(position));
    }

    /**
     * Gets the target angle of the pivot
     *
     * @return Target angle of the pivot
     */
    public Angle getPivotTargetAngle() {
        return targetPivotPosition;
    }

    /**
     * Checks if the wrist is on target
     *
     * @return True if the wrist is on target
     */
    public boolean pivotOnTarget() {
        return targetPivotPosition.isNear(CollectorConstants.TOLERANCE, getPivotAngle());
    }

    /**
     * dutycycleout command for shooter
     * @param power
     * @param position
     * @return the command for running the shooter
     */
    public Command collectCommand(double power, Angle position) {
        return new StartEndCommand(() -> deployCollector(power, position), () -> deployCollector(0, Degrees.of(0)), this);

    }

    /**
     * Get the angle of the pivot
     * @return angle of the pivot
     */
    public Angle getPivotAngle(){
        return pivotMotor.getPosition().getValue();
    }

    public void setPosition(Angle position) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPosition'");
    }
}
