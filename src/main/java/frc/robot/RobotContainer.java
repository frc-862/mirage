// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import static edu.wpi.first.units.Units.MetersPerSecond;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.LEDConstants;
import frc.robot.constants.LEDConstants.LED_STATES;
import frc.robot.constants.RobotMap;
import frc.robot.subsystems.Cannon;
import frc.robot.subsystems.Cannon.CannonConstants;
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
import frc.robot.subsystems.Turret.TurretConstants;
import frc.util.leds.Color;
import frc.util.leds.LEDBehaviorFactory;
import frc.util.leds.LEDBooleanSupplier;
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
                        VecBuilder.fill(-driver.getLeftY(), -driver.getLeftX()), DriveConstants.JOYSTICK_DEADBAND),
                        DriveConstants.CONTROLLER_POW).times(driver.getRightTriggerAxis() > DriveConstants.TRIGGER_DEADBAND 
                        ? DriveConstants.SLOW_MODE_MULT : 1.0), () -> MathUtil.copyDirectionPow(MathUtil.applyDeadband(-driver.getRightX(),
                        DriveConstants.JOYSTICK_DEADBAND), DriveConstants.CONTROLLER_POW)
                        * (driver.getRightTriggerAxis() > DriveConstants.TRIGGER_DEADBAND ? DriveConstants.SLOW_MODE_MULT : 1.0)));


        shooter.setDefaultCommand(shooter.coast());
        hood.setDefaultCommand(cannon.hoodAim());
        turret.setDefaultCommand(cannon.turretAim());
    }

    private void configureBindings() {

        /* Driver */
        new Trigger(driver::getXButton).whileTrue(drivetrain.brakeCommand());

        // new Trigger(driver::getYButton).whileTrue(turret.zero());
        
        // TODO: Bind OTF to LB and Climb AA to RB

        // change biases for the driver
        new Trigger(() -> driver.getPOV() == DriveConstants.DPAD_UP).onTrue(hood.changeBiasCommand(HoodConstants.BIAS_DELTA.unaryMinus()));
        new Trigger(() -> driver.getPOV() == DriveConstants.DPAD_DOWN).onTrue(hood.changeBiasCommand(HoodConstants.BIAS_DELTA));
        new Trigger(() -> driver.getPOV() == DriveConstants.DPAD_LEFT).onTrue(shooter.changeBiasCommand(ShooterConstants.BIAS_DELTA.unaryMinus()));
        new Trigger(() -> driver.getPOV() == DriveConstants.DPAD_RIGHT).onTrue(shooter.changeBiasCommand(ShooterConstants.BIAS_DELTA));

        // reset the field-centric heading
        new Trigger(() -> (driver.getStartButton() && driver.getBackButton()))
            .onTrue(drivetrain.resetFieldCentricCommand())
            .onTrue(leds.enableStateWithTimeout(LEDConstants.LED_STATES.SEED_FIELD_FORWARD.id(), 1));

        drivetrain.registerTelemetry(logger::telemeterize);

        new Trigger(() -> driver.getLeftTriggerAxis() > DriveConstants.TRIGGER_DEADBAND).whileTrue(drivetrain.robotCentricDrive(
            () -> MathUtil.copyDirectionPow(MathUtil.applyDeadband(
                    VecBuilder.fill(-driver.getLeftY(), -driver.getLeftX()), DriveConstants.JOYSTICK_DEADBAND), 
                    DriveConstants.CONTROLLER_POW).times(driver.getRightTriggerAxis() > DriveConstants.TRIGGER_DEADBAND ? 
                    DriveConstants.SLOW_MODE_MULT : 1.0), () -> MathUtil.copyDirectionPow(MathUtil.applyDeadband(-driver.getRightX(),
                    DriveConstants.JOYSTICK_DEADBAND), DriveConstants.CONTROLLER_POW)
                    * (driver.getRightTriggerAxis() > DriveConstants.TRIGGER_DEADBAND ? DriveConstants.SLOW_MODE_MULT : 1.0)));

        new Trigger(driver::getBButton).toggleOnTrue(turret.manual());

        /* Copilot */
        new Trigger(() -> drivetrain.isNearTrench()).whileTrue(hood.retract());
        new Trigger(copilot::getXButton).whileTrue(hood.retract().withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

        new Trigger(copilot::getLeftBumperButton).whileTrue(indexer.indexCommand(-IndexerConstants.SPINDEXDER_POWER,
            -IndexerConstants.TRANSFER_POWER));
        new Trigger(copilot::getRightBumperButton).whileTrue(indexer.indexCommand(IndexerConstants.SPINDEXDER_POWER,
            IndexerConstants.TRANSFER_POWER));

        new Trigger(() -> copilot.getBButton()).whileTrue(cannon.smartShoot()
            .alongWith(collector.collectCommand(() -> CollectorConstants.COLLECT_POWER * CollectorConstants.COLLECT_MULT))
            .deadlineFor(leds.enableState(LED_STATES.SHOOT.id())));

        new Trigger(copilot::getStartButton).whileTrue(collector.stowPivotCommand());
    
        new Trigger(() -> copilot.getRightTriggerAxis() > DriveConstants.TRIGGER_DEADBAND || copilot.getLeftTriggerAxis() > DriveConstants.TRIGGER_DEADBAND)
            .whileTrue(collector.collectCommand(() -> (copilot.getRightTriggerAxis() - copilot.getLeftTriggerAxis()) *  CollectorConstants.COLLECT_MULT));

        new Trigger(copilot::getBackButton).whileTrue(turret.manual()); // disable turret

        new Trigger(() -> Math.abs(copilot.getRightX()) > TurretConstants.MANUAL_CONTROL_DEADBAND).whileTrue(turret.setManualPowerCommand(() -> copilot.getRightX() * 0.1));

        // Temp Cand shots
        //RIGHT_, LEFT_, and MIDDLE_ are all set to 0, so temp shots wont work right now
        new Trigger(() -> copilot.getPOV() == DriveConstants.DPAD_RIGHT).whileTrue(cannon.createCandShotCommand(CannonConstants.RIGHT_SHOT).deadlineFor(rumble()));
        new Trigger(() -> copilot.getPOV() == DriveConstants.DPAD_LEFT).whileTrue(cannon.createCandShotCommand(CannonConstants.LEFT_SHOT).deadlineFor(rumble()));
        new Trigger(() -> copilot.getPOV() == DriveConstants.DPAD_UP).whileTrue(cannon.createCandShotCommand(CannonConstants.MIDDLE_SHOT).deadlineFor(rumble()));

        // new Trigger(() -> copilot.getPOV() == DriveConstants.DPAD_DOWN).whileTrue(
        //     shooter.shootCommand(() -> RotationsPerSecond.of(LightningShuffleboard.getDouble("Shooter", "RPS", 65)))
        //     .alongWith(hood.hoodCommand(() -> Degrees.of(LightningShuffleboard.getDouble("Hood", "Setpoint (Degrees)", 80))))
        //     .andThen(indexer.autoIndex(() -> LightningShuffleboard.getDouble("Indexer", "Power", IndexerConstants.SPINDEXDER_POWER), 
        //     () -> LightningShuffleboard.getDouble("Indexer", "Transfer Power", IndexerConstants.TRANSFER_POWER)))
        //     .finallyDo(shooter::stop));

        // new Trigger(() -> copilot.getPOV() == DriveConstants.DPAD_DOWN).whileTrue(hood.hoodCommand(() -> 
        //     Degrees.of(LightningShuffleboard.getDouble("Hood", "Setpoint (Degrees)", 80))));

        new Trigger(() -> (hood.isOnTarget() && shooter.isOnTarget() && turret.isOnTarget()))
        .whileTrue(new StartEndCommand(() -> copilot.setRumble(GenericHID.RumbleType.kBothRumble, 1d), () -> copilot.setRumble(GenericHID.RumbleType.kBothRumble, 0d)));

        // new Trigger(() -> DriverStation.isEnabled()).onTrue(hood.zeroCommand());

        new Trigger(copilot::getAButton).whileTrue(indexer.autoIndex(IndexerConstants.SPINDEXDER_POWER, IndexerConstants.TRANSFER_POWER)).onFalse(new InstantCommand(() -> indexer.stop()));

        new Trigger(() -> Math.abs(copilot.getLeftTriggerAxis()) > DriveConstants.JOYSTICK_DEADBAND).whileTrue(indexer.transferCommand(() -> copilot.getLeftTriggerAxis())).onFalse(new InstantCommand(() -> indexer.stopTransfer()));

        new Trigger(driver::getRightBumperButton).whileTrue(drivetrain.changeDrivetrainSupplyLimits());
    }
    ;
    private void configureNamedCommands() {
        NamedCommands.registerCommand("LED_SHOOT", leds.enableStateWithTimeout(LED_STATES.SHOOT.id(), 2));
        NamedCommands.registerCommand("LED_COLLECT", leds.enableStateWithTimeout(LED_STATES.COLLECT.id(), 2));
        NamedCommands.registerCommand("LED_CLIMB", leds.enableStateWithTimeout(LED_STATES.CLIMB.id(), 2));

        NamedCommands.registerCommand("MOVE_TO_TOWER", drivetrain.autoAlign(FieldConstants.getPose(FieldConstants.TOWER_POSITION)));
        NamedCommands.registerCommand("SMART_SHOOT", cannon.smartShoot().alongWith(collector.collectCommand(() -> CollectorConstants.COLLECT_POWER)));
        NamedCommands.registerCommand("COLLECT", collector.collectCommand(() -> CollectorConstants.COLLECT_POWER));
        NamedCommands.registerCommand("SMART_SHOOT", cannon.smartShoot().deadlineFor(leds.enableState(LED_STATES.SHOOT.id())));
        NamedCommands.registerCommand("COLLECT", collector.collectCommand(() -> CollectorConstants.COLLECT_POWER).deadlineFor(leds.enableState(LED_STATES.COLLECT.id())));
        NamedCommands.registerCommand("DEPLOY_COLLECTOR", collector.deployPivotCommand());
        NamedCommands.registerCommand("STOW_COLLECTOR", collector.stowPivotCommand());
        NamedCommands.registerCommand("WAIT_UNTIL_DEPLOYED", new WaitUntilCommand(collector::isDeployed));
        NamedCommands.registerCommand("WAIT_UNTIL_STOWED", new WaitUntilCommand(collector::isStowed));
        NamedCommands.registerCommand("COLLECTOR_WHEELS", collector.runCollectorWheels(() -> CollectorConstants.COLLECT_POWER));
        
        autoChooser = AutoBuilder.buildAutoChooser();
        LightningShuffleboard.send("Auton", "Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private void configureLeds() {
        leds.setDefaultBehavior(LEDBehaviorFactory.swirl(LEDConstants.stripAll, 10, 5, Color.ORANGE, Color.BLUE));

        leds.setBehavior(LED_STATES.TEST.id(), LEDBehaviorFactory.testStrip(LEDConstants.stripAll,
                () -> turret.getMaxLimitSwitch(),
                () -> turret.getZeroLimitSwitch(),
                () -> vision.getMacMiniConnection()
        ));
        leds.setBehavior(LED_STATES.TURRET_MANUAL.id(), LEDBehaviorFactory.blink(LEDConstants.stripAll, 2, Color.GREY));
        leds.setBehavior(LED_STATES.VISION_BAD.id(), LEDBehaviorFactory.solid(LEDConstants.stripUnderglow, Color.RED));
        leds.setBehavior(LED_STATES.TURRET_BAD.id(), LEDBehaviorFactory.pulse(LEDConstants.stripShooter, 2, Color.ORANGE));

        leds.setBehavior(LED_STATES.SEED_FIELD_FORWARD.id(), LEDBehaviorFactory.rainbow(LEDConstants.stripAll, 2));
        leds.setBehavior(LED_STATES.HOOD_STOWED.id(), LEDBehaviorFactory.solid(LEDConstants.stripShooter, Color.BLUE));

        leds.setBehavior(LED_STATES.SHOOT.id(), LEDBehaviorFactory.pulse(LEDConstants.stripAll, 2, Color.PURPLE));
        leds.setBehavior(LED_STATES.COLLECT.id(), LEDBehaviorFactory.pulse(LEDConstants.stripAll, 2, Color.YELLOW));
        leds.setBehavior(LED_STATES.CLIMB.id(), LEDBehaviorFactory.pulse(LEDConstants.stripAll, 2, Color.YELLOW));

        leds.setBehavior(LED_STATES.CANNED_SHOT_READY.id(), LEDBehaviorFactory.blink(LEDConstants.stripAll, 2, Color.GREEN));
        leds.setBehavior(LED_STATES.NEAR_HUB.id(), LEDBehaviorFactory.blink(LEDConstants.stripAll, 3, Color.RED));

        new Trigger(hood::isStowed).whileTrue(leds.enableState(LED_STATES.HOOD_STOWED.id()));

        new Trigger(turret::getManual).whileTrue(leds.enableState(LED_STATES.TURRET_MANUAL.id()));

        new Trigger(() -> !turret.getZeroed() && DriverStation.isDisabled()).whileTrue(leds.enableState(LED_STATES.TURRET_BAD.id()));

        new Trigger(new LEDBooleanSupplier(turret::getZeroed)).whileFalse(leds.enableState(LED_STATES.TURRET_BAD.id())); // turn off turret bad LED state once turret is zeroed

        // Turn off the "vision bad" LED state once the drivetrain has moved away from the origin, indicating we likely have a valid pose estimate.
        new Trigger(new LEDBooleanSupplier(() -> (DriverStation.isDisabled() && drivetrain.getPose().getTranslation().getDistance(new Translation2d()) < 0.1))).whileTrue(leds.enableState(LED_STATES.VISION_BAD.id()));

        new Trigger(new LEDBooleanSupplier(DriverStation::isDisabled)).whileTrue(leds.enableState(LED_STATES.TEST.id()));
        
        //Turn on the NEAR_HUB light when it is near the HUB.
        new Trigger(() -> cannon.isNearHub()).whileTrue(leds.enableState(LED_STATES.NEAR_HUB.id()));
    }

        /**
     * It waits until everything except turret is on target to start the rumble.
     * @return The command
     */
    public Command rumble(){
        return new SequentialCommandGroup(
            new WaitUntilCommand(() -> (hood.isOnTarget() && shooter.isOnTarget() && !turret.isOnTarget())),
            new StartEndCommand(() -> copilot.setRumble(GenericHID.RumbleType.kBothRumble, 1d), () -> copilot.setRumble(GenericHID.RumbleType.kBothRumble, 0d))
        );
    }

}
