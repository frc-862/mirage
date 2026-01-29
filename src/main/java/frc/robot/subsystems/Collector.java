package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.CollectorConstants;
import frc.robot.constants.RobotMap;
import frc.util.hardware.ThunderBird;
import frc.util.shuffleboard.LightningShuffleboard;

import static frc.util.Units.clamp;

public class Collector extends SubsystemBase {
    private final ThunderBird collectorMotor;
    private final ThunderBird pivotMotor;


    private LinearSystemSim<N1, N1, N1> collectorSim;
    private TalonFXSimState collectorMotorSim;

    private final DutyCycleOut collectorDutyCycle;

    private Angle targetPivotPosition = Degrees.of(0);
    private final PositionVoltage positionPID;

    /**
     * Creates a new Collector Subsystem.
     */
    public Collector() {
        collectorMotor = new ThunderBird(RobotMap.COLLECTOR, RobotMap.CAN_BUS,
            CollectorConstants.COLLECTOR_MOTOR_INVERTED, CollectorConstants.COLLECTOR_MOTOR_STATOR_LIMIT, CollectorConstants.COLLECTOR_MOTOR_BRAKE);

        pivotMotor = new ThunderBird(RobotMap.COLLECTOR_PIVOT, RobotMap.CAN_BUS,
            CollectorConstants.PIVOT_INVERTED, CollectorConstants.PIVOT_STATOR_LIMIT, CollectorConstants.PIVOT_BRAKE_MODE);






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


        pivotMotor.applyConfig(config);

        if (Robot.isSimulation()) {
            collectorSim = new LinearSystemSim<N1, N1, N1>(LinearSystemId.identifyVelocitySystem(CollectorConstants.COLLECTOR_SIM_kV,
                CollectorConstants.COLLECTOR_SIM_kA));
            collectorMotorSim = collectorMotor.getSimState();
            collectorMotorSim.setMotorType(MotorType.KrakenX60);
        }
    }

    @Override
    public void periodic() {}

    @Override
    public void simulationPeriodic() {
        collectorMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        collectorSim.setInput(collectorMotorSim.getMotorVoltageMeasure().in(Volts));
        collectorSim.update(Robot.kDefaultPeriod);

        collectorMotorSim.setRotorVelocity(collectorSim.getOutput(0));

        LightningShuffleboard.setDouble("Collector", "Velocity", getVelocity().in(RotationsPerSecond));
    }

    /**
     * Set the power of the collector motor using duty cycle out
     * @param power duty cycle value from -1.0 to 1.0
     */
    public void setCollectorPower(double power) {
        collectorMotor.setControl(collectorDutyCycle.withOutput(power));
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
     * Set the collector position in degrees
     *
     * @param position in degrees
     */
    public void setPosition(Angle position) {
        targetPivotPosition = clamp(position, CollectorConstants.MIN_ANGLE, CollectorConstants.MAX_ANGLE);
        pivotMotor.setControl(positionPID.withPosition(targetPivotPosition));
    }

    /**
     * Gets the target angle of the pivot
     *
     * @return Target angle of the pivot
     */
    public Angle getTargetAngle() {
        return targetPivotPosition;
    }

    public Angle getAngle(){
        return pivotMotor.getPosition().getValue();
    }


}
