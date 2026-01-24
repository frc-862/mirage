// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.hardware.ThunderBird;
import frc.util.shuffleboard.LightningShuffleboard;
import frc.robot.constants.RobotMap;
import frc.robot.constants.TurretConstants;
import frc.robot.Robot;
import frc.robot.constants.HoodConstants;

public class Hood extends SubsystemBase {

    public final DutyCycleOut dutyCycle = new DutyCycleOut(0.0);

    private ThunderBird hoodMotor;
    private SingleJointedArmSim hoodSim;
    private TalonFXSimState motorSim;
    private DCMotor gearbox;

    /** Creates a new Hood. */
    public Hood() {
        hoodMotor = new ThunderBird(RobotMap.HOOD_MOTOR_ID, RobotMap.CAN_BUS,
            HoodConstants.HOOD_MOTOR_INVERTED, HoodConstants.HOOD_MOTOR_STATOR_LIMIT,
            HoodConstants.HOOD_MOTOR_BRAKE_MODE);

        if (Robot.isSimulation()) {
            gearbox = DCMotor.getKrakenX44Foc(1);
            hoodSim = new SingleJointedArmSim(gearbox, HoodConstants.GEARING_RATIO,
                HoodConstants.MOI.magnitude(), HoodConstants.LENGTH.in(Meters),
                HoodConstants.MIN_ANGLE.in(Radians), HoodConstants.MAX_ANGLE.in(Radians),
                false, HoodConstants.MIN_ANGLE.in(Radians), 0d, 1d);

            motorSim = new TalonFXSimState(hoodMotor);

            motorSim.setRawRotorPosition(TurretConstants.MIN_ANGLE.in(Rotations));
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    /**
     * Sets power to (-1 to 1) to the hood motor.
     * @param power
     */
    public void setPower(double power) {
        hoodMotor.setControl(dutyCycle.withOutput(power));
    }
    /**
     * Stops the hood motor.
     */
    public void stop() {
        hoodMotor.stopMotor();
    }

        @Override
    public void simulationPeriodic() {
        double batteryVoltage = RobotController.getBatteryVoltage();
        motorSim.setSupplyVoltage(batteryVoltage);

        hoodSim.setInputVoltage(motorSim.getMotorVoltage());
        hoodSim.update(Robot.kDefaultPeriod);

        Angle simAngle = Rotations.of(hoodSim.getAngleRads());
        AngularVelocity simVeloc = RadiansPerSecond.of(hoodSim.getVelocityRadPerSec());

        motorSim.setRawRotorPosition(simAngle);
        motorSim.setRotorVelocity(simVeloc);

        LightningShuffleboard.setDouble("Hood", "Sim Angle", simAngle.in(Degrees));
        LightningShuffleboard.setDouble("Hood", "Sim Veloc", simVeloc.in(RadiansPerSecond));
        LightningShuffleboard.setDouble("Hood", "Power", dutyCycle.Output);
    }
}
