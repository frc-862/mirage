package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.CollectorConstants;
import frc.robot.constants.RobotMap;
import frc.util.hardware.ThunderBird;
import frc.util.shuffleboard.LightningShuffleboard;

public class Collector extends SubsystemBase {
    private ThunderBird collectMotor;
    private ThunderBird pivotMotor;
    private TalonFXSimState pivotSim;
    private SingleJointedArmSim collectorPivotSim;

    private double simMechanismPosition = 0.0; // Track position in rotations

    private final DutyCycleOut collectorDuty;

    private Angle targetPivotPosition = Degrees.of(0);
    private final PositionVoltage positionPID;

    private DCMotor gearbox;

    public Collector() {
        collectMotor = new ThunderBird(RobotMap.COLLECTOR_MOTOR_ID, RobotMap.CAN_BUS,
            CollectorConstants.COLLECTOR_MOTOR_INVERTED, CollectorConstants.COLLECTOR_MOTOR_STATOR_LIMIT, CollectorConstants.COLLECTOR_MOTOR_BRAKE_MODE);

        pivotMotor = new ThunderBird(RobotMap.PIVOT_MOTOR_ID, RobotMap.CAN_BUS,
            CollectorConstants.PIVOT_INVERTED, CollectorConstants.PIVOT_STATOR_LIMIT, CollectorConstants.PIVOT_BRAKE_MODE);

        collectorDuty = new DutyCycleOut(0.0);
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
            collectorPivotSim = new SingleJointedArmSim(gearbox, CollectorConstants.ROTOR_TO_ENCODER_RATIO, CollectorConstants.MOI.magnitude(),
            CollectorConstants.LENGTH.magnitude(), CollectorConstants.MIN_ANGLE.in(Radians), CollectorConstants.MAX_ANGLE.in(Radians), false,
            CollectorConstants.MIN_ANGLE.in(Radians), 0d,1d);

            pivotSim = pivotMotor.getSimState();
            pivotSim.setRawRotorPosition(CollectorConstants.MIN_ANGLE.in(Radians));
        }
    }

    @Override
    public void periodic() {
        pivotMotor.setControl(positionPID.withPosition(targetPivotPosition));
    }

    @Override
    public void simulationPeriodic(){
        pivotSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        Angle simAngle = Rotations.of(collectorPivotSim.getAngleRads());

        pivotSim.setRawRotorPosition(simAngle);

        // Get the motor voltage output
        // double motorVoltage = pivotSim.getMotorVoltage();

        // Update the physics simulation with voltage input
        // linearPivotSim.setInput(motorVoltage);
        // linearPivotSim.update(0.020); // 20ms period

        // pivotSim = new SingleJointedArmSim(null, motorVoltage, motorVoltage, motorVoltage, motorVoltage, motorVoltage, false, motorVoltage, null);

        // // Get simulated velocity (output of velocity system) in rotations/sec
        // double mechanismVelocity = linearPivotSim.getOutput(0);

        // Integrate velocity to get position
        // simMechanismPosition += mechanismVelocity * 0.020; // position in rotations

        // // Convert mechanism values to rotor values (multiply by gear ratio)
        // double rotorPosition = simMechanismPosition * CollectorConstants.ROTOR_TO_ENCODER_RATIO;
        // double rotorVelocity = mechanismVelocity * CollectorConstants.ROTOR_TO_ENCODER_RATIO;

        // Apply the simulated state back to the motor
        // pivotSim.setRawRotorPosition(rotorPosition);
        // pivotSim.setRotorVelocity(rotorVelocity);

        LightningShuffleboard.setDouble("Collector", "Collector Pivot", getPosition());
    }

    /**
     * Set the power of the motor using a duty cycle
     * @param power
     */
    public void setPower(double power) {
        collectMotor.setControl(collectorDuty.withOutput(power));
    }


    /**
     * Stops all movement to the collector motor
     */
    public void stop() {
        collectMotor.stopMotor();
    }

    /**
     * Set the collector position in degrees
     *
     * @param position in degrees
     */
    public void setPosition(Angle position) {
        //targetPivotPosition = clamp(position, CollectorConstants.MIN_ANGLE, CollectorConstants.MAX_ANGLE);
        pivotMotor.setControl(positionPID.withPosition(position));
        System.out.println("Set Position: " + getPosition());
    }

    /**
     * Gets the target angle of the pivot
     *
     * @return Target angle of the pivot
     */
    public Angle getTargetAngle() {
        return targetPivotPosition;
    }

    /**
     * Gets the current angle of the pivot
     *
     * @return Current angle of the pivot
     */
    public Angle getAngle() {
        return pivotMotor.getPosition().getValue();
    }

    public double getPosition(){
        return pivotMotor.getPosition().getValueAsDouble();
    }


    // /**
    //  * Checks if the wrist is on target
    //  *
    //  * @return True if the wrist is on target
    //  */
    // public boolean isOnTarget() {
    //     return targetPivotPosition.isNear(getPosition(), CollectorConstants.TOLERANCE);
    // }
}