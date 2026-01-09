// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Telemetry;

public class RobotContainer {

    private final XboxController driver;
    private final XboxController copilot;

    public final Swerve drivetrain;

    private final Telemetry logger;

    public RobotContainer() {
        driver = new XboxController(ControllerConstants.DRIVER_PORT);
        copilot = new XboxController(ControllerConstants.COPILOT_PORT);

        drivetrain = DriveConstants.createDrivetrain();

        logger = new Telemetry(DriveConstants.MaxSpeed);

        configureDefaultCommands();
        configureBindings();
    }

    private void configureDefaultCommands() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(drivetrain.controllerDrive(
            () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX(), 
            () -> driver.getRightBumperButton(), () -> driver.getLeftBumperButton()));
    }

    private void configureBindings() {
        new Trigger(driver::getXButton)
            .whileTrue(drivetrain.brakeCommand());

        // reset the field-centric heading
        new Trigger(() -> (driver.getStartButton() && driver.getBackButton()))
            .onTrue(drivetrain.resetFieldCentricCommand());

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
