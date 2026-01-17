// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.PoseBasedAutoAlign;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Swerve;
import frc.util.leds.Color;
import frc.util.leds.LEDBehaviorFactory;
import frc.util.leds.LEDSubsystem;
import frc.robot.constants.LEDConstants;
import frc.robot.constants.LEDConstants.LED_STATES;
import frc.robot.subsystems.Telemetry;
import frc.util.shuffleboard.LightningShuffleboard;

public class RobotContainer {

    private final XboxController driver;
    private final XboxController copilot;

    public final Swerve drivetrain;
    // public final Collector collector;
    public final LEDSubsystem leds;

    private final Telemetry logger;

    private SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        driver = new XboxController(ControllerConstants.DRIVER_PORT);
        copilot = new XboxController(ControllerConstants.COPILOT_PORT);

        drivetrain = DriveConstants.createDrivetrain();
        // collector = new Collector();

        logger = new Telemetry(DriveConstants.MaxSpeed.in(MetersPerSecond));
        leds = new LEDSubsystem(LED_STATES.values().length, LEDConstants.LED_COUNT, LEDConstants.LED_PWM_PORT);
        configureDefaultCommands();
        configureBindings();
        configureNamedCommands();
        configureLeds();
    }

    private void configureDefaultCommands() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(drivetrain.driveCommand(() -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));
    }

    private void configureBindings() {
        new Trigger(driver::getXButton)
            .whileTrue(drivetrain.brakeCommand()
                .deadlineFor(leds.enableState(LED_STATES.BRAKE.id()))
            );

        // reset the field-centric heading
        new Trigger(() -> (driver.getStartButton() && driver.getBackButton()))
            .onTrue(drivetrain.resetFieldCentricCommand());

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureNamedCommands(){
        autoChooser = AutoBuilder.buildAutoChooser();
        LightningShuffleboard.send("Auton", "Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private void configureLeds() {
        leds.setDefaultBehavior(LEDBehaviorFactory.swirl(LEDConstants.stripAll, 10, 5, Color.ORANGE, Color.BLUE));

        leds.setBehavior(LED_STATES.TEST.id(), LEDBehaviorFactory.testStrip(LEDConstants.stripAll,
            () -> false,
            () -> true
        ));
        leds.setBehavior(LED_STATES.ERROR.id(), LEDBehaviorFactory.blink(LEDConstants.stripAll, 2, Color.RED));
        leds.setBehavior(LED_STATES.AUTO.id(), LEDBehaviorFactory.rainbow(LEDConstants.stripAll, 2));
        leds.setBehavior(LED_STATES.BRAKE.id(), LEDBehaviorFactory.solid(LEDConstants.stripAll, Color.GREEN));

        new Trigger(DriverStation:: isTest).whileTrue(leds.enableState(LED_STATES.TEST.id()));

        new Trigger(() -> DriverStation.isAutonomous() && DriverStation.isEnabled()).whileTrue(leds.enableState(LED_STATES.AUTO.id()));
    }
}
