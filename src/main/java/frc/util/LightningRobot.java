package frc.util;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.util.testing.SystemTest;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.Properties;

/**
 * Base robot class, provides
 * {@link frc.thunder.fault.FaultMonitor fault monitoring} and loops with
 * varying
 * periods {@link LightningRobot#robotBackgroundPeriodic() background},
 * {@link LightningRobot#robotLowPriorityPeriodic() low}, and
 * {@link LightningRobot#robotMediumPriorityPeriodic() medium} priority
 * loops.
 *
 * Uses {@link frc.thunder.auto.Autonomous} to configure autonomous
 * commands. Also includes
 * self-testing support with
 * {@link frc.thunder.testing.SystemTestCommand}.
 */
public class LightningRobot extends TimedRobot {

    private LightningContainer container;

    private final static double SETTLE_TIME = 3.0;
    private final static double LOOP_TIME = 0.02;

    private int counter = 0;

    private int medPriorityFreq = (int) Math.round(0.1 / getPeriod());

    private int lowPriorityFreq = (int) Math.round(1 / getPeriod());

    private int backgroundPriorityFreq = (int) Math.round(10 / getPeriod());

    private Command autonomousCommand;

    public LightningRobot(LightningContainer container) {
        // timed robot has default constructor of 20ms and we want it at 10ms.
        super(LOOP_TIME);
        this.container = container;
    }

    public double getSettleTime() {
        return SETTLE_TIME;
    }

    /**
     * Getter for the configured robot container.
     * 
     * @return the {@link frc.thunder.LightningContainer} for the robot.
     */
    public LightningContainer getContainer() {
        return container;
    }

    public boolean haveDriverStation = false;

    /**
     * Nothing should happen here.
     */
    @Override
    public void disabledPeriodic() {
        if (!haveDriverStation && DriverStation.getAlliance().isPresent()) {
            haveDriverStation = true;
            allianceKnown(DriverStation.getAlliance().get());
            System.out.println("SET FORWARD");
        }
    }

    protected void allianceKnown(Alliance alliance) {
    }

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     *
     * If you override it, be sure to call super.robotInit
     */
    @Override
    public void robotInit() {
        System.out.println("LightningRobot.robotInit");

        // Starts WPILIB data logging
        final var envLogPath = System.getenv("LOG_PATH");
        if (envLogPath != null) {
            DataLogManager.start(envLogPath);
        } else if (Paths.get("/u/logs").toFile().exists()) {
            DataLogManager.start("/u/logs");
        } else {
            DataLogManager.start("/home/lvuser/logs");
        }

        // Start logging driverstation
        DriverStation.startDataLog(DataLogManager.getLog());

        // No Live Window for now
        LiveWindow.disableAllTelemetry();

        // Note our start time
        System.out.println("Starting time: " + Timer.getFPGATimestamp());

        // Read our version properties
        try {
            Properties props = new Properties();
            var stream = ClassLoader.getSystemResourceAsStream("version.properties");
            if (stream != null) {
                props.load(stream);
                System.out.println("Version: " + props.getProperty("VERSION_NAME", "n/a"));
                System.out.println("Build: " + props.getProperty("VERSION_BUILD", "n/a"));
                System.out.println("Built at: " + props.getProperty("BUILD_TIME", "n/a"));
                System.out.println("Git branch: " + props.getProperty("GIT_BRANCH", "n/a"));
                System.out.println("Git hash: " + props.getProperty("GIT_HASH", "n/a"));
                System.out.println("Git status: " + props.getProperty("BUILD_STATUS", "n/a"));
            }
        } catch (IOException e) {
            System.out.println("Unable to read build version information.");
        }

        // Load our system tests to the dashboard
        SystemTest.loadTests();
    }

    /**
     * This function is called every robot packet, no matter the mode. Use
     * this for items like diagnostics that you want ran during disabled,
     * autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before
     * LiveWindow and Shuffleboard integrated updating.
     *
     * If you override this method, be sure to call super.robotPeriod() as
     * it drives our lower priority loops, datalogging, fault monitoring,
     * etc.
     */
    @Override
    public void robotPeriodic() {
        double time = Timer.getFPGATimestamp();
        if (time > SETTLE_TIME) {
            counter += 1;
            if (counter % medPriorityFreq == 0) {
                robotMediumPriorityPeriodic();
            }
            if (counter % lowPriorityFreq == 0) {
                robotLowPriorityPeriodic();
            }
            if (counter % backgroundPriorityFreq == 0) {
                robotBackgroundPeriodic();
            }

        }

        CommandScheduler.getInstance().run();
    }

    /**
     * A slower loop, running once every 10 seconds
     *
     * Note as currently implemented it still needs to
     * complete in our loop time or it delay higher
     * priority opterations. If you have a low priority,
     * long running operation, consider creating a background
     * thread.
     */
    protected void robotBackgroundPeriodic() {
    }

    /**
     * A slow loop, running once a second
     *
     * Note as currently implemented it still needs to
     * complete in our loop time or it delay higher
     * priority opterations. If you have a low priority,
     * long running operation, consider creating a background
     * thread.
     */
    protected void robotLowPriorityPeriodic() {
    }

    /**
     * A loop, running 10 times a second
     *
     * Note as currently implemented it still needs to
     * complete in our loop time or it delay higher
     * priority opterations. If you have a low priority,
     * long running operation, consider creating a background
     * thread.
     */
    protected void robotMediumPriorityPeriodic() {
    }

    /**
     * The default implementation handles getting the selected command
     * from Shuffleboard.
     *
     * If you override this method, be sure to call {@code super.autonomousInit()}
     * or
     * the selected registered command will not be executed.
     */
    @Override
    public void autonomousInit() {
        System.out.println("LightningRobot.autonomousInit");
        autonomousCommand = container.getAutonomousCommand();
        if (autonomousCommand != null)
            autonomousCommand.schedule();
    }

    /**
     * The default implementation handles canceling the autonomous command.
     *
     * If you override this method, be sure to call {@code super.teleopInit()} or
     * the autonomous command will not be canceled when teleop starts.
     * 
     * Alternatively, if you want the autonomous command to finish running
     * into teleop, you may override this method w/o calling
     * {@code super.teleopInit()}
     */
    @Override
    public void teleopInit() {
        System.out.println("LightningRobot.teleopInit");
        if (autonomousCommand != null)
            autonomousCommand.cancel();
    }

    @Override
    public void testInit() {
        System.out.println("LightningRobot.testInit");
    }

    @Override
    public void testPeriodic() {
    }

    /**
     * The default implementation configures the default commands in the event they
     * have
     * been disabled by {@link LightningRobot#testInit()}.
     */
    @Override
    public void disabledInit() {
        System.out.println("LightningRobot.disabledInit");
        getContainer().configureDefaultCommands();
    }

}