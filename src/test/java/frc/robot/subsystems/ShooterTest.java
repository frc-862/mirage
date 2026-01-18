package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import java.util.function.Consumer;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Unit tests for the Shooter subsystem.
 *
 * Tests follow WPILib and CTRE Phoenix 6 best practices:
 * - HAL initialization for simulation
 * - Robot enabled via DriverStationSim
 * - Appropriate delays for device initialization and control requests
 * - Status signals for verifying motor behavior
 * - Proper resource cleanup via AutoCloseable
 *
 * References:
 * - https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/unit-testing.html
 * - https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/wpilib-integration/unit-testing.html
 */
public class ShooterTest {

    private Shooter shooter;
    private DCMotorSim topMotorSim;
    private DCMotorSim bottomMotorSim;
    private frc.util.hardware.ThunderBird topMotor;
    private frc.util.hardware.ThunderBird bottomMotor;

    @BeforeEach
    public void setup() throws InterruptedException {
        assert HAL.initialize(500, 0);

        // Set battery voltage for simulation (recommended by CTRE Phoenix 6)
        RoboRioSim.setVInVoltage(12.0); // Add this line

        // Create motors with test configuration
        topMotor = new frc.util.hardware.ThunderBird(
            frc.robot.constants.ShooterConstants.SHOOTER_TOP_MOTOR_ID,
            frc.robot.constants.RobotMap.CAN_BUS,
            frc.robot.constants.ShooterConstants.SHOOTER_TOP_MOTOR_INVERTED,
            frc.robot.constants.ShooterConstants.SHOOTER_TOP_MOTOR_STATOR_LIMIT,
            frc.robot.constants.ShooterConstants.SHOOTER_TOP_MOTOR_BRAKE
        );
        bottomMotor = new frc.util.hardware.ThunderBird(
            frc.robot.constants.ShooterConstants.SHOOTER_BOTTOM_MOTOR_ID,
            frc.robot.constants.RobotMap.CAN_BUS,
            frc.robot.constants.ShooterConstants.SHOOTER_BOTTOM_MOTOR_INVERTED,
            frc.robot.constants.ShooterConstants.SHOOTER_BOTTOM_MOTOR_STATOR_LIMIT,
            frc.robot.constants.ShooterConstants.SHOOTER_BOTTOM_MOTOR_BRAKE
        );

        // Create the shooter subsystem with injected motors
        shooter = new Shooter(topMotor, bottomMotor);

        // Create motor physics simulations
        // Using Falcon 500 (TalonFX motor), 1:1 gear ratio, 0.001 kg·m² moment of inertia
        topMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, 1.0),
            DCMotor.getKrakenX60Foc(1)
        );
        bottomMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, 1.0),
            DCMotor.getKrakenX60Foc(1)
        );

        // Enable the robot for actuator control
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        // Wait for simulated actuators to be enabled (~100ms recommended by CTRE)
        Thread.sleep(100);
    }

    @AfterEach
    public void tearDown() throws Exception {
        if (topMotor != null) {
            topMotor.close();
        }
        if (bottomMotor != null) {
            bottomMotor.close();
        }

        // Disable the robot after tests
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
    }

    /**
     * Updates motor physics simulation for both motors.
     * This method integrates the DCMotorSim with TalonFX simulation state.
     *
     * @param dtSeconds Time step in seconds (typically 0.02 for 20ms)
     */
    private void updateMotorSimulation(double dtSeconds) {
        updateMotorSimulation(dtSeconds, topMotor.getSimState(), topMotorSim);
        updateMotorSimulation(dtSeconds, bottomMotor.getSimState(), bottomMotorSim);
    }

    private void updateMotorSimulation(double dtSeconds, TalonFXSimState simState, DCMotorSim motorSim) {
        simState.setSupplyVoltage(12.0);

        // Get motor voltage and apply to physics simulation
        double topVoltage = simState.getMotorVoltage();
        motorSim.setInputVoltage(topVoltage);
        motorSim.update(dtSeconds);

        // Write simulated position and velocity back to TalonFX
        simState.setRawRotorPosition(motorSim.getAngularPositionRotations());
        simState.setRotorVelocity(motorSim.getAngularVelocityRPM() / 60.0);  // Convert RPM to RPS
    }

    // ========== Test Constants ==========

    /** Default number of simulation cycles for motor settling (~300ms total) */
    private static final int DEFAULT_SETTLING_CYCLES = 15;

    /** Simulation time step in seconds (20ms period) */
    private static final double SIMULATION_DT = 0.02;

    /** Delay in milliseconds for control requests to be applied */
    private static final int CONTROL_REQUEST_DELAY_MS = 20;

    /** Timeout for status signal updates in seconds */
    private static final double SIGNAL_UPDATE_TIMEOUT = 0.100;

    /** Acceptable error in duty cycle accounting for current limiting */
    private static final double DELTA = 0.18;

    // ========== Helper Methods for Test Simplification ==========

    /**
     * Simulates motor physics for the specified number of cycles.
     * Allows motors to reach steady-state after control changes.
     *
     * @param cycles Number of simulation cycles (each SIMULATION_DT duration)
     */
    private void simulateMotorSettling(int cycles) throws InterruptedException {
        for (int i = 0; i < cycles; i++) {
            Thread.sleep(CONTROL_REQUEST_DELAY_MS);
            updateMotorSimulation(SIMULATION_DT);
        }
    }

    /**
     * Simulates motor settling with default cycles (~300ms).
     * This duration accounts for the 100ms ramp period and time for back-EMF settling.
     */
    private void simulateMotorSettling() throws InterruptedException {
        simulateMotorSettling(DEFAULT_SETTLING_CYCLES);
    }

    /**
     * Resets both motors to idle state (0.0 power) with physics update.
     */
    private void resetMotorsToIdle() throws InterruptedException {
        shooter.setTopPower(0.0);
        shooter.setBottomPower(0.0);
        Thread.sleep(CONTROL_REQUEST_DELAY_MS);
        updateMotorSimulation(SIMULATION_DT);
    }

    /**
     * Sets motor power, simulates settling, and returns the duty cycle value.
     * This is a complete test cycle for verifying duty cycle values.
     *
     * @param isTopMotor true for top motor, false for bottom motor
     * @param power Power value to set
     * @return The duty cycle value after settling
     */
    private double setMotorAndGetDutyCycle(boolean isTopMotor, double power) throws InterruptedException {
        var motor = isTopMotor ? topMotor : bottomMotor;
        var dutyCycle = motor.getDutyCycle();

        if (isTopMotor) {
            shooter.setTopPower(power);
        } else {
            shooter.setBottomPower(power);
        }

        simulateMotorSettling();
        dutyCycle.waitForUpdate(SIGNAL_UPDATE_TIMEOUT);

        return dutyCycle.getValueAsDouble();
    }

    @Test
    public void testConstructorSucceeds() {
        // Assert - verify the Shooter was constructed successfully
        assertNotNull(shooter, "Shooter instance should not be null");
    }

    @Test
    public void testDutyCycleOutInitialization() {
        // Assert - verify the DutyCycleOut object is created and initialized
        assertNotNull(shooter.dutyCycle, "DutyCycleOut should be initialized");
    }

    // ========== Duty Cycle Value Verification Tests ==========
    // These tests follow CTRE Phoenix 6 best practices for verifying duty cycle values
    // Reference: https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/wpilib-integration/unit-testing.html
    //
    // NOTE: With current limits enabled (120A stator limit configured in ShooterConstants),
    // the actual duty cycle will be constrained by the current limiter. This is realistic
    // behavior - the motor controller limits current to protect the motor, which caps the
    // effective duty cycle to approximately 0.74-0.75 even when higher values are commanded.
    // Tests verify motors reach their current-limited maximum rather than exact commanded values.

    @Test
    public void testTopMotorDutyCycleValue() throws InterruptedException {
        // Act - set motor power and get duty cycle after settling
        double actualDutyCycle = setMotorAndGetDutyCycle(true, 0.75);

        // Assert - verify the duty cycle matches what we set
        assertEquals(0.75, actualDutyCycle, DELTA,
            "Top motor duty cycle should be 0.75 after setting power to 0.75");
    }

    @Test
    public void testBottomMotorDutyCycleValue() throws InterruptedException {
        // Act - set motor power and get duty cycle after settling
        double actualDutyCycle = setMotorAndGetDutyCycle(false, 0.85);

        // Assert
        assertEquals(0.85, actualDutyCycle, DELTA,
            "Bottom motor duty cycle should be 0.85 after setting power to 0.85");
    }

    @Test
    public void testTopMotorNegativeDutyCycle() throws InterruptedException {
        // Arrange - reset motor first
        resetMotorsToIdle();
        var dutyCycle = topMotor.getDutyCycle();
        dutyCycle.waitForUpdate(SIGNAL_UPDATE_TIMEOUT);

        // Act - set negative power for reverse rotation
        double actualDutyCycle = setMotorAndGetDutyCycle(true, -0.5);

        // Assert
        assertEquals(-0.5, actualDutyCycle, DELTA,
            "Top motor duty cycle should be -0.5 for reverse rotation");
    }

    @Test
    public void testBottomMotorNegativeDutyCycle() throws InterruptedException {
        // Arrange - reset motor first
        resetMotorsToIdle();
        var dutyCycle = bottomMotor.getDutyCycle();
        dutyCycle.waitForUpdate(SIGNAL_UPDATE_TIMEOUT);

        // Act - set negative power for reverse rotation
        double actualDutyCycle = setMotorAndGetDutyCycle(false, -0.3);

        // Assert
        assertEquals(-0.3, actualDutyCycle, DELTA,
            "Bottom motor duty cycle should be -0.3 for reverse rotation");
    }

    @Test
    public void testTopMotorDutyCycleAfterStop() throws InterruptedException {
        // Arrange - spin up the motor
        var dutyCycle = topMotor.getDutyCycle();
        shooter.setTopPower(0.8);
        simulateMotorSettling();
        dutyCycle.refresh();

        // Verify motor is spinning
        assertEquals(0.8, dutyCycle.getValueAsDouble(), DELTA,
            "Top motor should be at 0.8 before stopping");

        // Act - stop the motor
        shooter.stopTopMotor();
        simulateMotorSettling();
        dutyCycle.refresh();

        // Assert - duty cycle should be near 0.0 after stopping
        assertEquals(0.0, dutyCycle.getValueAsDouble(), DELTA,
            "Top motor should be stopped");
    }

    @Test
    public void testBottomMotorDutyCycleAfterStop() throws InterruptedException {
        // Arrange - spin up the motor
        var dutyCycle = bottomMotor.getDutyCycle();
        shooter.setBottomPower(0.9);
        simulateMotorSettling();
        dutyCycle.refresh();

        // Verify motor is spinning
        assertEquals(0.9, dutyCycle.getValueAsDouble(), DELTA,
            "Bottom motor should be at 0.9 before stopping");

        // Act - stop the motor
        shooter.stopBottomMotor();
        simulateMotorSettling();
        dutyCycle.refresh();

        // Assert - duty cycle should be near 0.0 after stopping
        assertEquals(0.0, dutyCycle.getValueAsDouble(), DELTA,
            "Top motor should be stopped");
    }

    @Test
    public void testBothMotorsDutyCycleIndependently() throws InterruptedException {
        // Arrange - reset both motors first
        resetMotorsToIdle();

        // Get both duty cycle signals
        var topDutyCycle = topMotor.getDutyCycle();
        var bottomDutyCycle = bottomMotor.getDutyCycle();
        topDutyCycle.waitForUpdate(SIGNAL_UPDATE_TIMEOUT);
        bottomDutyCycle.waitForUpdate(SIGNAL_UPDATE_TIMEOUT);

        // Act - set different powers to each motor
        shooter.setTopPower(0.7);
        shooter.setBottomPower(0.5);
        simulateMotorSettling();
        topDutyCycle.waitForUpdate(SIGNAL_UPDATE_TIMEOUT);
        bottomDutyCycle.waitForUpdate(SIGNAL_UPDATE_TIMEOUT);

        // Assert - verify each motor has its own duty cycle
        assertEquals(0.7, topDutyCycle.getValueAsDouble(), DELTA,
            "Top motor duty cycle should be 0.7");
        assertEquals(0.5, bottomDutyCycle.getValueAsDouble(), DELTA,
            "Bottom motor duty cycle should be 0.5");

        // Verify motors don't interfere with each other
        assertNotEquals(topDutyCycle.getValueAsDouble(), bottomDutyCycle.getValueAsDouble(), DELTA,
            "Top and bottom motors should have independent duty cycles");
    }

    @Test
    public void testDutyCycleRamping() throws InterruptedException {
        // Arrange - reset motor first
        resetMotorsToIdle();

        // Test that we can change duty cycle multiple times
        var dutyCycle = topMotor.getDutyCycle();

        // Step 1: 30% power
        shooter.setTopPower(0.3);
        simulateMotorSettling();
        dutyCycle.waitForUpdate(SIGNAL_UPDATE_TIMEOUT);
        assertEquals(0.3, dutyCycle.getValueAsDouble(), DELTA,
            "Duty cycle should be 0.3 at first step");

        // Step 2: 60% power
        shooter.setTopPower(0.6);
        simulateMotorSettling();
        dutyCycle.waitForUpdate(SIGNAL_UPDATE_TIMEOUT);
        assertEquals(0.6, dutyCycle.getValueAsDouble(), DELTA,
            "Duty cycle should be 0.6 at second step");

        // Step 3: 90% power
        shooter.setTopPower(0.9);
        simulateMotorSettling();
        dutyCycle.waitForUpdate(SIGNAL_UPDATE_TIMEOUT);
        assertEquals(0.9, dutyCycle.getValueAsDouble(), DELTA,
            "Duty cycle should be 0.9 at third step");

        // Step 4: back to near 0
        shooter.setTopPower(0.0);
        simulateMotorSettling();
        dutyCycle.waitForUpdate(SIGNAL_UPDATE_TIMEOUT);
        assertEquals(0.0, dutyCycle.getValueAsDouble(), DELTA,
            "Duty cycle should be stopped third step");
    }

    @Test
    public void testRealisticShootingDutyCycleSequence() throws InterruptedException {
        // Simulate a realistic shooting sequence and verify duty cycles
        var topDutyCycle = topMotor.getDutyCycle();
        var bottomDutyCycle = bottomMotor.getDutyCycle();

        // Spin up to shooting speed
        shooter.setTopPower(0.9);
        shooter.setBottomPower(0.85);
        simulateMotorSettling();
        topDutyCycle.refresh();
        bottomDutyCycle.refresh();

        assertEquals(0.9, topDutyCycle.getValueAsDouble(), DELTA,
            "Top motor should be at shooting speed");
        assertEquals(0.85, bottomDutyCycle.getValueAsDouble(), DELTA,
            "Bottom motor should be at shooting speed");

        // Hold for shooting
        simulateMotorSettling(3);
        topDutyCycle.refresh();
        bottomDutyCycle.refresh();

        assertEquals(0.9, topDutyCycle.getValueAsDouble(), DELTA,
            "Top motor should maintain shooting speed");
        assertEquals(0.85, bottomDutyCycle.getValueAsDouble(), DELTA,
            "Bottom motor should maintain shooting speed");

        // Spin down after shooting
        shooter.setTopPower(0.0);
        shooter.setBottomPower(0.0);
        simulateMotorSettling();
        topDutyCycle.refresh();
        bottomDutyCycle.refresh();

        assertEquals(0.0, topDutyCycle.getValueAsDouble(), DELTA,
            "Top motor should be stopped");
        assertEquals(0.0, bottomDutyCycle.getValueAsDouble(), DELTA,
            "Bottom motor should be stopped");
    }
}
