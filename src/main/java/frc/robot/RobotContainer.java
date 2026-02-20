// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.PoseBasedAutoAlign;
import frc.robot.commands.ShooterAim;
import frc.robot.commands.TurretAim;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.LEDConstants;
import frc.robot.constants.LEDConstants.LED_STATES;
import frc.robot.constants.RobotMap;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Collector.CollectorConstants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Indexer.IndexerConstants;
import frc.robot.subsystems.Swerve.FieldConstants;
import frc.robot.subsystems.MapleSim;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Telemetry;
import frc.robot.subsystems.Turret;
import frc.util.leds.Color;
import frc.util.leds.LEDBehaviorFactory;
import frc.util.leds.LEDSubsystem;
import frc.util.shuffleboard.LightningShuffleboard;

public class RobotContainer {

    private final XboxController driver;
    private final XboxController copilot;

    private final Swerve drivetrain;

    // TODO: make final after subsystems are added
    private Collector collector;
    private Indexer indexer;
    private Turret turret;
    private Hood hood;
    private Shooter shooter;
    private final LEDSubsystem leds;

    private final Telemetry logger;

    private SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        driver = new XboxController(RobotMap.DRIVER_PORT);
        copilot = new XboxController(RobotMap.COPILOT_PORT);

        drivetrain = DriveConstants.createDrivetrain();

        logger = new Telemetry(DriveConstants.MaxSpeed.in(MetersPerSecond));
        leds = new LEDSubsystem(LED_STATES.values().length, LEDConstants.LED_COUNT, LEDConstants.LED_PWM_PORT);

        if (RobotMap.IS_OASIS || Robot.isSimulation()) {
            collector = new Collector();
            indexer = new Indexer();
            shooter = new Shooter();
            hood = new Hood();
            turret = new Turret(drivetrain);
            new PhotonVision(drivetrain);
        }

        if (Robot.isSimulation()) {
            new MapleSim(drivetrain, collector, indexer, turret, hood, shooter);
        }

        configureDefaultCommands();
        configureBindings();
        configureNamedCommands();
        configureLeds();
    }

    private void configureDefaultCommands() {
        /* Driver */
        /*
        * Note that X is defined as forward according to WPILib convention,
        * Y is defined as to the left according to WPILib convention.
        */
        drivetrain.setDefaultCommand(drivetrain.driveCommand(
                () -> MathUtil.copyDirectionPow(MathUtil.applyDeadband(
                        VecBuilder.fill(-driver.getLeftY(), -driver.getLeftX()), RobotMap.CONTROLLER_DEADBAND)
                        .times(driver.getRightBumperButton() ? DriveConstants.SLOW_MODE_MULT : 1.0),
                        RobotMap.CONTROLLER_POW), () -> MathUtil.copyDirectionPow(MathUtil.applyDeadband(-driver.getRightX(),
                        RobotMap.CONTROLLER_DEADBAND), RobotMap.CONTROLLER_POW) * (driver.getRightBumperButton()
                ? DriveConstants.SLOW_MODE_MULT : 1.0)));

        if (RobotMap.IS_OASIS || Robot.isSimulation()) {
            indexer.setDefaultCommand(indexer.indexRunCommand(() -> (copilot.getRightTriggerAxis() - copilot.getLeftTriggerAxis())));
            shooter.setDefaultCommand(shooter.coast());
            hood.setDefaultCommand(hood.hoodAim(drivetrain));
            // turret.setDefaultCommand(new TurretAim(drivetrain, turret));
        }
    }

    private void configureBindings() {
        /* Driver */
        new Trigger(driver::getXButton)
            .whileTrue(drivetrain.brakeCommand()
                    .deadlineFor(leds.enableState(LED_STATES.BRAKE.id()))
            );

        // reset the field-centric heading
        new Trigger(() -> (driver.getStartButton() && driver.getBackButton()))
            .onTrue(drivetrain.resetFieldCentricCommand());

        drivetrain.registerTelemetry(logger::telemeterize);

        new Trigger(driver::getLeftBumperButton).whileTrue(drivetrain.robotCentricDrive(
            () -> MathUtil.copyDirectionPow(MathUtil.applyDeadband(
                    VecBuilder.fill(-driver.getLeftY(), -driver.getLeftX()), RobotMap.CONTROLLER_DEADBAND)
                    .times(driver.getRightBumperButton() ? DriveConstants.SLOW_MODE_MULT : 1.0),
                    RobotMap.CONTROLLER_POW), () -> MathUtil.copyDirectionPow(MathUtil.applyDeadband(-driver.getRightX(),
                    RobotMap.CONTROLLER_DEADBAND), RobotMap.CONTROLLER_POW) * (driver.getRightBumperButton()
            ? DriveConstants.SLOW_MODE_MULT : 1.0)));

        /* Copilot */
            if (Robot.isSimulation()) {
            new Trigger(driver::getAButton).whileTrue(hood.run(() -> hood.setPosition(Degrees.of(60))));

            new Trigger(driver::getYButton).onTrue(new InstantCommand(() -> { // VERY TEMPORARY
                shooter.setVelocity(RotationsPerSecond.of(100));
                indexer.setSpindexerPower(1d);
            })).onFalse(new InstantCommand(() -> {
                shooter.stop();
                indexer.stop();
            }));

            new Trigger(copilot::getBButton).whileTrue(hood.run(() -> hood.setPosition(hood.getTargetAngle().plus(Degrees.of(0.5)))));

            new Trigger(copilot::getXButton).whileTrue(hood.run(() -> hood.setPosition(hood.getTargetAngle().minus(Degrees.of(0.5)))));

            new Trigger(() -> copilot.getPOV() == 270).onTrue(new InstantCommand(() -> hood.changeBias(Hood.HoodConstants.BIAS_DELTA.unaryMinus()))); // Assign a trigger soon
            new Trigger(() -> copilot.getPOV() == 90).onTrue(new InstantCommand(() -> hood.changeBias(Hood.HoodConstants.BIAS_DELTA))); // Assign a trigger soon
            new Trigger(() -> copilot.getPOV() == 0).onTrue(new InstantCommand(() -> shooter.changeBias(Shooter.ShooterConstants.BIAS_DELTA)));

            new Trigger(() -> copilot.getPOV() == 180).onTrue(new InstantCommand(() -> shooter.changeBias(Shooter.ShooterConstants.BIAS_DELTA.unaryMinus())));

            new Trigger(driver::getBButton).onTrue(new TurretAim(drivetrain, turret, Swerve.FieldConstants.getTargetData(Swerve.FieldConstants.GOAL_POSITION)));
        }

        if (RobotMap.IS_OASIS) {
            new Trigger(copilot::getLeftBumperButton).whileTrue(collector.collectCommand(-CollectorConstants.COLLECT_POWER));
            new Trigger(copilot::getRightBumperButton).whileTrue(collector.collectCommand(CollectorConstants.COLLECT_POWER));

            // Temp Bindings for testing purposes

            new Trigger(copilot::getYButton).whileTrue(shooter.shootCommand(RotationsPerSecond.of(65))
                .andThen(indexer.indexCommand(IndexerConstants.SPINDEXDER_POWER, IndexerConstants.TRANSFER_POWER))
                .finallyDo(shooter::stop));

            new Trigger(copilot::getAButton).whileTrue(
                shooter.shootCommand(() -> RotationsPerSecond.of(LightningShuffleboard.getDouble("Shooter", "RPS", 65)))
                .alongWith(hood.hoodCommand(() -> Degrees.of(LightningShuffleboard.getDouble("Hood", "Setpoint (Degrees)", 80))))
                .andThen(indexer.indexCommand(() -> LightningShuffleboard.getDouble("Indexer", "Power", IndexerConstants.SPINDEXDER_POWER), 
                () -> LightningShuffleboard.getDouble("Indexer", "Transfer Power", IndexerConstants.TRANSFER_POWER)))
                .finallyDo(shooter::stop));

            new Trigger(copilot::getBButton).whileTrue(new TurretAim(drivetrain, turret));
        }
    }
    private void configureNamedCommands(){
        NamedCommands.registerCommand("LED_SHOOT", leds.enableStateWithTimeout(LED_STATES.SHOOT.id(), 2));
        NamedCommands.registerCommand("LED_COLLECT", leds.enableStateWithTimeout(LED_STATES.COLLECT.id(), 2));
        NamedCommands.registerCommand("LED_CLIMB", leds.enableStateWithTimeout(LED_STATES.CLIMB.id(), 2));

        NamedCommands.registerCommand("SCORE_PRELOADED", new InstantCommand()); //Temporary Command. Planning on using smart shoot.
        NamedCommands.registerCommand("MOVE_TO_TOWER", new PoseBasedAutoAlign(drivetrain, () -> FieldConstants.TOWER_POSITION.getPose()));
        NamedCommands.registerCommand("SHOOT_PRELOADED", new ShooterAim(shooter, drivetrain, hood, turret, indexer));


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
        leds.setBehavior(LED_STATES.VISION_BAD.id(), LEDBehaviorFactory.solid(LEDConstants.stripAll, Color.RED));
        leds.setBehavior(LED_STATES.BRAKE.id(), LEDBehaviorFactory.solid(LEDConstants.stripAll, Color.GREEN));
        leds.setBehavior(LED_STATES.SHOOT.id(), LEDBehaviorFactory.pulse(LEDConstants.stripAll, 2, Color.ORANGE));
        leds.setBehavior(LED_STATES.COLLECT.id(), LEDBehaviorFactory.pulse(LEDConstants.stripAll, 2, Color.BLUE));
        leds.setBehavior(LED_STATES.CLIMB.id(), LEDBehaviorFactory.pulse(LEDConstants.stripAll, 2, Color.YELLOW));
        leds.setBehavior(LED_STATES.CANNED_SHOT_START.id(), LEDBehaviorFactory.blink(LEDConstants.stripAll, 2, Color.YELLOW));
        leds.setBehavior(LED_STATES.CANNED_SHOT_READY.id(), LEDBehaviorFactory.blink(LEDConstants.stripAll, 2, Color.GREEN));

        new Trigger(DriverStation::isTest).whileTrue(leds.enableState(LED_STATES.TEST.id()));

        new Trigger(() -> DriverStation.isAutonomous() && DriverStation.isEnabled()).whileTrue(leds.enableState(LED_STATES.AUTO.id()));

        // At startup, turn on the "vision bad" LED state to indicate that the pose/vision estimate may be unreliable.
        leds.setState(LED_STATES.VISION_BAD.id(), true);
        // Turn off the "vision bad" LED state once the drivetrain has moved away from the origin, indicating we likely have a valid pose estimate.
        new Trigger(() -> (DriverStation.isEnabled() || drivetrain.getPose().getTranslation().getDistance(new Translation2d()) > 0.1)).onTrue(new InstantCommand(() -> leds.setState(LED_STATES.VISION_BAD.id(), false)));
    }
}
