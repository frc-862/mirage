package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.CollectorConstants;
import frc.robot.constants.RobotMap;
import frc.util.hardware.ThunderBird;

public class Collector extends SubsystemBase {
    private ThunderBird collectMotor;
    private ThunderBird pivotMotor;
    private CANcoder encoder;

    private final DutyCycleOut collectorDuty = new DutyCycleOut(0);

    private double targetPivotPosition = 0;
    private final PositionVoltage positionPID = new PositionVoltage(0);

    public Collector() {
        collectMotor = new ThunderBird(RobotMap.COLLECTOR_MOTOR_ID, RobotMap.CAN_BUS,
            CollectorConstants.COLLECTOR_MOTOR_INVERTED, CollectorConstants.COLLECTOR_MOTOR_STATOR_LIMIT, CollectorConstants.COLLECTOR_MOTOR_BRAKE_MODE);

        pivotMotor = new ThunderBird(RobotMap.PIVOT_MOTOR_ID, RobotMap.CAN_BUS,
            CollectorConstants.PIVOT_INVERTED, CollectorConstants.PIVOT_STATOR_LIMIT, CollectorConstants.PIVOT_BRAKE_MODE);

        encoder = new CANcoder(RobotMap.PIVOT_ENCODER_ID, RobotMap.CAN_BUS);
        CANcoderConfiguration angleConfig = new CANcoderConfiguration();
        angleConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5d;
        angleConfig.MagnetSensor.MagnetOffset = Robot.isReal() ? CollectorConstants.PIVOT_OFFSET : 0d;
        angleConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        encoder.getConfigurator().apply(angleConfig);

        TalonFXConfiguration config = pivotMotor.getConfig();
        config.Slot0.kP = CollectorConstants.PIVOT_KP;
        config.Slot0.kI = CollectorConstants.PIVOT_KI;
        config.Slot0.kD = CollectorConstants.PIVOT_KD;
        config.Slot0.kS = CollectorConstants.PIVOT_KS;
        config.Slot0.kV = CollectorConstants.PIVOT_KV;
        config.Slot0.kA = CollectorConstants.PIVOT_KA;
        config.Slot0.kG = CollectorConstants.PIVOT_KG;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        config.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.SensorToMechanismRatio = CollectorConstants.ENCODER_TO_MECHANISM_RATIO;
        config.Feedback.RotorToSensorRatio = CollectorConstants.ROTOR_TO_ENCODER_RATIO;

        pivotMotor.applyConfig(config);
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
    public void setPosition(double position) {
        targetPivotPosition = MathUtil.clamp(position, CollectorConstants.MIN_ANGLE.in(Degrees), CollectorConstants.MAX_ANGLE.in(Degrees));
        pivotMotor.setControl(positionPID.withPosition(Units.degreesToRotations(position)));
    }

    /**
     * Gets the target angle of the pivot
     *
     * @return Target angle of the pivot
     */
    public double getTargetAngle() {
        return targetPivotPosition;
    }

    /**
     * Gets the current angle of the pivot
     *
     * @return Current angle of the pivot
     */
    public double getAngle() {
        return encoder.getAbsolutePosition().getValue().in(Degrees);
    }

    /**
     * Checks if the wrist is on target
     *
     * @return True if the wrist is on target
     */
    public boolean isOnTarget() {
        return Math.abs(targetPivotPosition - getAngle()) < CollectorConstants.TOLERANCE;
    }
}