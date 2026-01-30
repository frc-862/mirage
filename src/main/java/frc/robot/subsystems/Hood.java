// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.sim.TalonFXSimState;


import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.Units;
import frc.util.hardware.ThunderBird;
import frc.util.shuffleboard.LightningShuffleboard;
import frc.robot.constants.RobotMap;
import frc.robot.Robot;
import frc.robot.constants.HoodConstants;

public class Hood extends SubsystemBase {
    private ThunderBird hoodMotor;

    public final DutyCycleOut dutyCycle;

    final PositionVoltage request;
    private Angle targetAngle;

    private SingleJointedArmSim hoodSim;
    private TalonFXSimState motorSim;
    private DCMotor gearbox;

    /** Creates a new Hood Subsystem. */
    public Hood() {
        hoodMotor = new ThunderBird(RobotMap.HOOD, RobotMap.CAN_BUS,
            HoodConstants.INVERTED, HoodConstants.STATOR_LIMIT,
            HoodConstants.BRAKE);

        dutyCycle = new DutyCycleOut(0d);

        request = new PositionVoltage(0d);

        var talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kP = HoodConstants.kP;
        slot0Configs.kI = HoodConstants.kI;
        slot0Configs.kD = HoodConstants.kD;
        slot0Configs.kS = HoodConstants.kS;
        slot0Configs.kV = HoodConstants.kV;
        slot0Configs.kA = HoodConstants.kA;

        hoodMotor.applyConfig(talonFXConfigs);

        if (Robot.isSimulation()) {
            gearbox = DCMotor.getKrakenX44Foc(1);
            hoodSim = new SingleJointedArmSim(gearbox, HoodConstants.GEARING_RATIO,
                HoodConstants.MOI.magnitude(), HoodConstants.LENGTH.in(Meters),
                HoodConstants.MIN_ANGLE.in(Radians), HoodConstants.MAX_ANGLE.in(Radians),
                true, HoodConstants.MIN_ANGLE.in(Radians), 0d, 1d);

            motorSim = new TalonFXSimState(hoodMotor);

            motorSim.setRawRotorPosition(HoodConstants.MIN_ANGLE.in(Rotations));
        }
    }

    @Override
    public void periodic() {}

    /**
     * Set the power of the hood motor using duty cycle out
     * @param power duty cycle value from -1.0 to 1.0
     */
    public void setPower(double power) {
        hoodMotor.setControl(dutyCycle.withOutput(power));
    }

    /**
     * Sets position of the hood
     * @param position in degrees
     */
    public void setPosition(Angle position) {
        targetAngle = Units.clamp(position, HoodConstants.MIN_ANGLE, HoodConstants.MAX_ANGLE);

        hoodMotor.setControl(request.withPosition(targetAngle));
    }

    /**
     * Gets the current angle of the hood
     * @return
     * current angle
     */
    public Angle getAngle() {
        return hoodMotor.getPosition().getValue();
    }

    /**
     * Gets the target angle of the hood
     * @return
     * targetAngle
     */
    public Angle getTargetAngle() {
        return targetAngle;
    }

    /**
     * Returns true if the hood is on target
     * @return
     * True if on target, false otherwise
     */
    public boolean isOnTarget() {
        return getAngle().isNear(getTargetAngle(), HoodConstants.POSITION_TOLERANCE);
    }

    /**
     * Stops all movement to the hood motor
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

        Angle simAngle = Radians.of(hoodSim.getAngleRads());
        AngularVelocity simVeloc = RadiansPerSecond.of(hoodSim.getVelocityRadPerSec());

        motorSim.setRawRotorPosition(simAngle);
        motorSim.setRotorVelocity(simVeloc);

        LightningShuffleboard.setDouble("Hood", "Sim Angle", simAngle.in(Degrees));
    }
}
