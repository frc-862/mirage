package frc.util;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link LightningRobot} periodic methods (other
 * than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared in a subclass of this
 * class.
 */
public abstract class LightningContainer {

    public LightningContainer() {
        initializeHardware();

        initializeNamedCommands();

        // Setup driver commands and defaults
        configureButtonBindings();
        configureDefaultCommands();
        configureLEDs();
        // initializeDashboardCommands();

        // Setup fault monitoring
        // configureFaultCodes();
        // configureFaultMonitors();

        // Configure System Tests
        // configureSystemTests()/;

    }

    /**
     * Instantialize subsystems/static methods
     */
    protected abstract void initializeHardware();

    /**
     * Instantialize named commands
     */
    protected abstract void initializeNamedCommands();

    /**
     * Connects commands with buttons on joysticks
     */
    protected abstract void configureButtonBindings();

    /**
     * Registers all systems tests for the robot to be run
     */
    // protected abstract void configureSystemTests();

    /**
     * Configures all default commands to run on subsystems when no other command
     * requires that subsystem
     */
    protected abstract void configureDefaultCommands();

    protected abstract void configureLEDs();

    /**
     * Cancles all default commands
     */
    // protected abstract void releaseDefaultCommands();

    /**
     * Puts command buttons on the dashboard
     */
    // protected abstract void initializeDashboardCommands();

    /**
     * Configures all robot-specific fault codes
     */
    // protected abstract void configureFaultCodes();

    /**
     * Configures listeners for fault codes
     */
    // protected abstract void configureFaultMonitors();

    /**
     * Gets output from sendable chooser
     */
    protected abstract Command getAutonomousCommand();
}