package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;

import frc.util.hardware.ThunderBird;
import frc.util.shuffleboard.LightningShuffleboard;

public class Collector {
    private ThunderBird motor;

    private double targetPower = 0;

    public void periodic() {
        LightningShuffleboard.setDouble("Collector", "current", motor.getStatorCurrent().getValueAsDouble());
        LightningShuffleboard.setDouble("Diagnostic", "COLLECTOR Temperature", motor.getDeviceTemp().getValueAsDouble());
    }

    /**
     * Set the power of the motor using a duty cycle
     * @param power
     */
    public void setPower(double power) {
        motor.setControl(new DutyCycleOut(power));
        targetPower = power; // store the current power for reference
    }

    public void stop() {
        motor.stopMotor();
        targetPower = 0; // reset the current power since we stopped the motor
    }

    /**
     * @return current velocity of collector rot/sec
     */
    public double getVelocity() {
        return motor.getRotorVelocity().getValueAsDouble();
    }

    public double getPower() {
        return motor.getMotorVoltage().getValueAsDouble();
    }

    /**
     * @return the last power set to the motor. This may not be the actual power due to motor control loops
     */
    public double getTargetPower() {
        // this returns the last power set to the motor
        // this may not be the actual power due to motor control loops
        return targetPower;
    }
}
