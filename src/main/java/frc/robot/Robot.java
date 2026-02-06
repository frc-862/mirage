// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command autonomousCommand;

    private static RobotContainer robotContainer;

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
        if(!edu.wpi.first.wpilibj.RobotBase.isSimulation()){
            DataLogManager.start("/home/lvuser/logs");
            DriverStation.startDataLog(DataLogManager.getLog());
        }

        //Silence joystick warnings in simulation
        if (edu.wpi.first.wpilibj.RobotBase.isSimulation()){
            DriverStation.silenceJoystickConnectionWarning(true);
        }
        
        // No Live Window for now
        LiveWindow.disableAllTelemetry();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
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
    public void teleopExit() {}

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
