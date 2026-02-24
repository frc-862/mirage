// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.util.shuffleboard.LightningShuffleboard;

public class Robot extends TimedRobot {
    private Command autonomousCommand;

    private static RobotContainer robotContainer;

    private DoubleLogEntry totalCurrentEntry;
    private DoubleLogEntry voltageEntry;

    public Robot() {
        getContainer();
    }

    public static RobotContainer getContainer() {
        if (robotContainer == null) {
            robotContainer = new RobotContainer();
        }

        return robotContainer;
    }

    @Override
    public void robotInit() {
        // Only start WPILIB data logging on the real robot
        if(!isSimulation()){
            DataLogManager.start("/home/lvuser/logs");
            DriverStation.startDataLog(DataLogManager.getLog());
        }

        //Silence joystick warnings in simulation
        if (isSimulation()){
            DriverStation.silenceJoystickConnectionWarning(true);
        }
        
        // No Live Window for now
        LiveWindow.disableAllTelemetry();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        if (DriverStation.isFMSAttached()) {

            if (totalCurrentEntry == null) {
                totalCurrentEntry = new DoubleLogEntry(DataLogManager.getLog(), "PDH/TotalCurrent");
                voltageEntry = new DoubleLogEntry(DataLogManager.getLog(), "PDH/Voltage");
            }

            totalCurrentEntry.append(getContainer().pdh.getTotalCurrent());
            voltageEntry.append(getContainer().pdh.getVoltage());
        } else {
            LightningShuffleboard.setDouble("PDH", "Total Current", getContainer().pdh.getTotalCurrent());
            LightningShuffleboard.setDouble("PDH", "Voltage", getContainer().pdh.getVoltage());
        }
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {
        LightningShuffleboard.setBool("Drive Team", "Auton Set", false); // reset auton set for next match
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
