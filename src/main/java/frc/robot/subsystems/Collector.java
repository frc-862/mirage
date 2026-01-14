package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotMap;
import frc.util.hardware.ThunderBird;

public class Collector extends SubsystemBase{
    private static final double DEFAULT_COLLECTOR_POWER = 0.8;
    private ThunderBird motor;

    public Collector () {
        this.motor = new ThunderBird(RobotMap.COLLECTOR_MOTOR_ID, RobotMap.CAN_BUS,
        RobotMap.COLLECTOR_MOTOR_INVERTED, RobotMap.COLLECTOR_MOTOR_STATOR_LIMIT, RobotMap.COLLECTOR_MOTOR_BRAKE_MODE);
    }

    /**
     * Set the power of the motor using a duty cycle
     * @param power
     */
    public void setPower(double power) {
        motor.setControl(new DutyCycleOut(power));
    }

    public void stop() {
        motor.stopMotor();
    }

    public Command runCollector(double power) {
        return new StartEndCommand(
            () -> setPower(power),
            this::stop,
            this
        );
    }

    public Command runCollector() {
        return runCollector(DEFAULT_COLLECTOR_POWER);
    }

    public Command reverseCollector() {
        return runCollector(-DEFAULT_COLLECTOR_POWER);
    }
}
