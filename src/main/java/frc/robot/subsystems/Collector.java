package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotMap;
import frc.util.hardware.ThunderBird;

public class Collector extends SubsystemBase {
    private ThunderBird collectMotor;

    private ThunderBird pivotMotor;

    public Collector() {
        collectMotor = new ThunderBird(RobotMap.COLLECTOR_MOTOR_ID, RobotMap.CAN_BUS,
            RobotMap.COLLECTOR_MOTOR_INVERTED, RobotMap.COLLECTOR_MOTOR_STATOR_LIMIT, RobotMap.COLLECTOR_MOTOR_BRAKE_MODE);

        pivotMotor = new ThunderBird(RobotMap.COLLECTOR_PIVOT_ID, RobotMap.CAN_BUS,
            RobotMap.COLLECTOR_PIVOT_INVERTED, RobotMap.COLLECTOR_PIVOT_STATOR_LIMIT, RobotMap.COLLECTOR_PIVOT_BRAKE_MODE);

        TalonFXConfiguration config = pivotMotor.getConfig();
        config.Slot0.kP = 

    }

    /**
     * Set the power of the motor using a duty cycle
     * @param power
     */
    public void setPower(double power) {
        collectMotor.setControl(new DutyCycleOut(power));
    }

    /**
     * Stops all movement to the collector motor
     */
    public void stop() {
        collectMotor.stopMotor();
    }
}
