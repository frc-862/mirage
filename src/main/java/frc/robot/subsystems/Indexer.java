// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.RobotMap;
import frc.util.hardware.ThunderBird;

public class Indexer extends SubsystemBase {
    private final ThunderBird indexerMotor;
    private final DutyCycleOut dutyCycle;

    private final LinearSystemSim<N2, N1, N2> indexerSim;

    /** Creates a new Indexer. */
    public Indexer() {
        indexerMotor = new ThunderBird(RobotMap.INDEXER_MOTOR_ID, RobotMap.CAN_BUS,
            IndexerConstants.INDEXER_MOTOR_INVERTED, IndexerConstants.INDEXER_MOTOR_STATOR_LIMIT, IndexerConstants.INDEXER_MOTOR_BRAKE_MODE);

        dutyCycle = new DutyCycleOut(0);

        if (Robot.isSimulation()){
            indexerSim = new LinearSystemSim<N2, N1, N2>(LinearSystemId.createDCMotorSystem(0.3, 0.3));
        }
    }


    @Override
    public void simulationPeriodic() {
        indexerMotor.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
        indexerSim.setInput(indexerMotor.getSimState().getMotorVoltage());
        indexerSim.update(0.02);
    }

    /**
     * Sets power to (-1 to 1) to the indexer motor.
     * @param power
     */
    public void setPower(double power) {
        indexerMotor.setControl(dutyCycle.withOutput(power));
    }

    /**
     * Stops the indexer motor.
     */
    public void stop() {
        indexerMotor.stopMotor();
    }
}
