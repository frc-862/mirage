// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.constants.DriveConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.LEDConstants;
import frc.robot.constants.LEDConstants.LED_STATES;
import frc.robot.constants.RobotMap;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Cannon;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Collector.CollectorConstants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hood.HoodConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Indexer.IndexerConstants;
import frc.robot.subsystems.MapleSim;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Telemetry;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Cannon.CannonConstants;
import frc.robot.subsystems.Cannon;
import frc.robot.commands.PoseBasedAutoAlign;
import frc.util.leds.Color;
import frc.util.leds.LEDBehaviorFactory;
import frc.util.leds.LEDSubsystem;
import frc.util.shuffleboard.LightningShuffleboard;


public class RobotContainer {

    private final XboxController driver;
    private final XboxController copilot;

    private final Swerve drivetrain;
    private final Collector collector;
    private final Indexer indexer;
    private final Turret turret;
    private final Hood hood;
    private final Shooter shooter;
    private final LEDSubsystem leds;
    public final PowerDistribution pdh;
    @SuppressWarnings("unused")
    private final PhotonVision vision;
    private final Telemetry logger;
    private final Cannon cannon;

    private SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        driver = new XboxController(RobotMap.DRIVER_PORT);
        copilot = new XboxController(RobotMap.COPILOT_PORT);

        drivetrain = DriveConstants.createDrivetrain();

        logger = new Telemetry(DriveConstants.MaxSpeed.in(MetersPerSecond));
        leds = new LEDSubsystem(LED_STATES.values().length, LEDConstants.LED_COUNT, LEDConstants.LED_PWM_PORT);
        pdh = new PowerDistribution(RobotMap.PDH, ModuleType.kRev);

        collector = new Collector();
        indexer = new Indexer();
        shooter = new Shooter();
        hood = new Hood();
        turret = new Turret(drivetrain);
        cannon = new Cannon(shooter, turret, hood, drivetrain, indexer);
        vision = new PhotonVision(drivetrain);
        

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
                        VecBuilder.fill(-driver.getLeftY(), -driver.getLeftX()), DriveConstants.JOYSTICK_DEADBAND)
                        .times(driver.getRightTriggerAxis() > DriveConstants.TRIGGER_DEADBAND ? DriveConstants.SLOW_MODE_MULT : 1.0),
                        DriveConstants.CONTROLLER_POW), () -> MathUtil.copyDirectionPow(MathUtil.applyDeadband(-driver.getRightX(),
                        DriveConstants.JOYSTICK_DEADBAND), DriveConstants.CONTROLLER_POW)
                        * (driver.getRightTriggerAxis() > DriveConstants.TRIGGER_DEADBAND ? DriveConstants.SLOW_MODE_MULT : 1.0)));


        /*
         * Copilot Default Commands
         */
        collector.setDefaultCommand(collector.collectRunCommand(() -> copilot.getRightTriggerAxis() - copilot.getLeftTriggerAxis()));
        shooter.setDefaultCommand(shooter.coast());

        hood.setDefaultCommand(cannon.hoodAim());
        turret.setDefaultCommand(cannon.turretAim());
    }

    private void configureBindings() {

        /* Driver */
        new Trigger(driver::getXButton)
            .whileTrue(drivetrain.brakeCommand()
                .deadlineFor(leds.enableState(LED_STATES.BRAKE.id()))
            );

        // TODO: Bind OTF to LB and Climb AA to RB

        /*
         * change biases for the driver
         */
        new Trigger(() -> driver.getPOV() == DriveConstants.DPAD_UP).onTrue(hood.changeBiasCommand(HoodConstants.BIAS_DELTA.unaryMinus()));
        new Trigger(() -> driver.getPOV() == DriveConstants.DPAD_DOWN).onTrue(hood.changeBiasCommand(HoodConstants.BIAS_DELTA));
        new Trigger(() -> driver.getPOV() == DriveConstants.DPAD_LEFT).onTrue(shooter.changeBiasCommand(ShooterConstants.BIAS_DELTA.unaryMinus()));
        new Trigger(() -> driver.getPOV() == DriveConstants.DPAD_RIGHT).onTrue(shooter.changeBiasCommand(ShooterConstants.BIAS_DELTA));

        // reset the field-centric heading
        new Trigger(() -> (driver.getStartButton() && driver.getBackButton()))
            .onTrue(drivetrain.resetFieldCentricCommand());

        drivetrain.registerTelemetry(logger::telemeterize);

        new Trigger(() -> driver.getLeftTriggerAxis() > DriveConstants.TRIGGER_DEADBAND).whileTrue(drivetrain.robotCentricDrive(
            () -> MathUtil.copyDirectionPow(MathUtil.applyDeadband(
                    VecBuilder.fill(-driver.getLeftY(), -driver.getLeftX()), DriveConstants.JOYSTICK_DEADBAND)
                    .times(driver.getRightTriggerAxis() > DriveConstants.TRIGGER_DEADBAND ? DriveConstants.SLOW_MODE_MULT : 1.0),
                    DriveConstants.CONTROLLER_POW), () -> MathUtil.copyDirectionPow(MathUtil.applyDeadband(-driver.getRightX(),
                    DriveConstants.JOYSTICK_DEADBAND), DriveConstants.CONTROLLER_POW)
                    * (driver.getRightTriggerAxis() > DriveConstants.TRIGGER_DEADBAND ? DriveConstants.SLOW_MODE_MULT : 1.0)));

        new Trigger(driver::getAButton).whileTrue(cannon.run(() -> // temp for interp map tuning
            LightningShuffleboard.setDouble("Cannon", "Target Distance", cannon.getTargetDistance().in(Meters))));

        /* Copilot */
        new Trigger(() -> drivetrain.isNearTrench())
            .whileTrue(hood.retract());
        new Trigger(copilot::getXButton).whileTrue(hood.retract());

        new Trigger(copilot::getLeftBumperButton).whileTrue(indexer.indexCommand(-IndexerConstants.SPINDEXDER_POWER,
            -IndexerConstants.TRANSFER_POWER));
        new Trigger(copilot::getRightBumperButton).whileTrue(indexer.indexCommand(IndexerConstants.SPINDEXDER_POWER,
            IndexerConstants.TRANSFER_POWER));

        new Trigger(() -> copilot.getBButton()).whileTrue(cannon.smartShoot());

        new Trigger(() -> copilot.getRightTriggerAxis() > DriveConstants.TRIGGER_DEADBAND).whileTrue(
            collector.pivotCommand(CollectorConstants.DEPLOY_ANGLE));
        new Trigger(copilot::getYButton).whileTrue(collector.pivotCommand(CollectorConstants.STOWED_ANGLE));

        new Trigger(copilot::getBackButton).whileTrue(turret.idle().beforeStarting(turret::stop)); // disable turret

        // Temp Cand shots
        //RIGHT_, LEFT_, and MIDDLE_ are all set to 0, so temp shots wont work right now
        new Trigger(() -> copilot.getPOV() == DriveConstants.DPAD_RIGHT).whileTrue(cannon.createCandShotCommand(CannonConstants.RIGHT_SHOT));
        new Trigger(() -> copilot.getPOV() == DriveConstants.DPAD_LEFT).whileTrue(cannon.createCandShotCommand(CannonConstants.LEFT_SHOT));
        new Trigger(() -> copilot.getPOV() == DriveConstants.DPAD_UP).whileTrue(cannon.createCandShotCommand(CannonConstants.MIDDLE_SHOT));
    }
    
    private void configureNamedCommands() {
        NamedCommands.registerCommand("LED_SHOOT", leds.enableStateWithTimeout(LED_STATES.SHOOT.id(), 2));
        NamedCommands.registerCommand("LED_COLLECT", leds.enableStateWithTimeout(LED_STATES.COLLECT.id(), 2));
        NamedCommands.registerCommand("LED_CLIMB", leds.enableStateWithTimeout(LED_STATES.CLIMB.id(), 2));

        NamedCommands.registerCommand("MOVE_TO_TOWER", new PoseBasedAutoAlign(drivetrain, FieldConstants.getPose(FieldConstants.TOWER_POSITION)));
        NamedCommands.registerCommand("SMART_SHOOT", cannon.smartShoot());
        NamedCommands.registerCommand("COLLECT", collector.collectCommand(CollectorConstants.COLLECT_POWER));
        NamedCommands.registerCommand("DEPLOY_COLLECTOR", collector.collectCommand(CollectorConstants.DEPLOY_POWER, CollectorConstants.DEPLOY_ANGLE));
        NamedCommands.registerCommand("STOW_COLLECTOR", collector.collectCommand(CollectorConstants.HOLD_POWER, CollectorConstants.STOWED_ANGLE));

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
        leds.setBehavior(LED_STATES.NOT_READY_FOR_MATCH.id(), LEDBehaviorFactory.blink(LEDConstants.stripAll, 2, Color.PURPLE));
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
        new Trigger(() -> (DriverStation.isEnabled() || drivetrain.getPose().getTranslation().getDistance(new Translation2d()) > 0.1))
            .onTrue(new InstantCommand(() -> leds.setState(LED_STATES.VISION_BAD.id(), false))
            .ignoringDisable(true));

        new Trigger(() -> DriverStation.isFMSAttached()).onTrue(new InstantCommand(() -> // only set "not ready for match" if we're connected to the field, to avoid confusion during testing
            leds.setState(LED_STATES.NOT_READY_FOR_MATCH.id(), true)).ignoringDisable(true));

        new Trigger(() -> DriverStation.isEnabled() || (turret.getZeroLimitSwitch() 
            && LightningShuffleboard.getBool("Drive Team", "Auton Set", false)) && DriverStation.isFMSAttached())
            .onTrue(new InstantCommand(() -> leds.setState(LED_STATES.NOT_READY_FOR_MATCH.id(), false))
            .ignoringDisable(true));
    }
}
