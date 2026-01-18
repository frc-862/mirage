package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import java.util.EnumMap;
import java.util.Map;
import java.util.function.DoubleConsumer;
import java.util.stream.Stream;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class ShooterTest {
  private static final double BATTERY_VOLTAGE = 12.0;
  private static final double DT = 0.02;          // 20ms
  private static final double DUTY_TOL = 0.01;
  private static final double DEFAULT_TIMEOUT_S = 0.5;

  private Shooter shooter;

  private enum WhichMotor { TOP, BOTTOM }

  /**
   * Bundles motor + control hooks + sim objects.
   *
   * @param motor The ThunderBird motor controller
   * @param setPower Function to set motor power
   * @param stop Function to stop the motor
   * @param simState TalonFX simulation state
   * @param sim DC motor physics simulation
   */
  private record MotorUnderTest(
      frc.util.hardware.ThunderBird motor,
      DoubleConsumer setPower,
      Runnable stop,
      TalonFXSimState simState,
      DCMotorSim sim
  ) {
    void updateSim(double dtSeconds, double supplyVoltage) {
      simState.setSupplyVoltage(supplyVoltage);

      var motorVoltage = simState.getMotorVoltage();
      sim.setInputVoltage(motorVoltage);
      sim.update(dtSeconds);

      simState.setRawRotorPosition(sim.getAngularPositionRotations());
      simState.setRotorVelocity(sim.getAngularVelocityRPM() / 60.0); // RPM -> RPS
    }
  }

  private final Map<WhichMotor, MotorUnderTest> motors = new EnumMap<>(WhichMotor.class);

  @BeforeEach
  void setup() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");

    RoboRioSim.setVInVoltage(BATTERY_VOLTAGE);

    var top = new frc.util.hardware.ThunderBird(
        frc.robot.constants.ShooterConstants.SHOOTER_TOP_MOTOR_ID,
        frc.robot.constants.RobotMap.CAN_BUS,
        frc.robot.constants.ShooterConstants.SHOOTER_TOP_MOTOR_INVERTED,
        frc.robot.constants.ShooterConstants.SHOOTER_TOP_MOTOR_STATOR_LIMIT,
        frc.robot.constants.ShooterConstants.SHOOTER_TOP_MOTOR_BRAKE
    );

    var bottom = new frc.util.hardware.ThunderBird(
        frc.robot.constants.ShooterConstants.SHOOTER_BOTTOM_MOTOR_ID,
        frc.robot.constants.RobotMap.CAN_BUS,
        frc.robot.constants.ShooterConstants.SHOOTER_BOTTOM_MOTOR_INVERTED,
        frc.robot.constants.ShooterConstants.SHOOTER_BOTTOM_MOTOR_STATOR_LIMIT,
        frc.robot.constants.ShooterConstants.SHOOTER_BOTTOM_MOTOR_BRAKE
    );

    shooter = new Shooter(top, bottom);

    motors.put(WhichMotor.TOP, new MotorUnderTest(
        top,
        shooter::setTopPower,
        shooter::stopTopMotor,
        top.getSimState(),
        newKrakenSim()
    ));
    motors.put(WhichMotor.BOTTOM, new MotorUnderTest(
        bottom,
        shooter::setBottomPower,
        shooter::stopBottomMotor,
        bottom.getSimState(),
        newKrakenSim()
    ));

    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    // CTRE often recommends ~100ms after enable for devices/signals to settle.
    runForSeconds(0.10);
    resetAllMotors();
  }

  @AfterEach
  void tearDown() throws Exception {
    // Close motors (MotorUnderTest holds ThunderBird)
    for (var m : motors.values()) {
      if (m.motor() != null) {
        m.motor().close();
      }
    }
    motors.clear();

    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }

  private static DCMotorSim newKrakenSim() {
    // 1 motor, 1:1 ratio, 0.001 kg·m² inertia (same values you used)
    var motor = DCMotor.getKrakenX60Foc(1);
    return new DCMotorSim(
        LinearSystemId.createDCMotorSystem(motor, 0.001, 1.0),
        motor
    );
  }

  private MotorUnderTest motor(WhichMotor which) {
    var m = motors.get(which);
    assertNotNull(m, "Missing motor: " + which);
    return m;
  }

  private void stepSim() {
    // If you truly need CTRE async breathing room, keep it *small* and isolated.
    // Otherwise, remove this sleep and see if your tests remain stable.
    try {
      Thread.sleep(5);
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
    }

    for (var m : motors.values()) {
      m.updateSim(DT, BATTERY_VOLTAGE);
    }
    DriverStationSim.notifyNewData();
  }

  private void runForSeconds(double seconds) {
    int cycles = (int) Math.ceil(seconds / DT);
    for (int i = 0; i < cycles; i++) {
      stepSim();
    }
  }

  private void resetAllMotors() {
    shooter.setTopPower(0.0);
    shooter.setBottomPower(0.0);
    stepSim();
  }

  private void assertDutyCycleEventually(MotorUnderTest m, double expected, double tol, double timeoutSec) {
    var sig = m.motor().getDutyCycle();
    int cycles = (int) Math.ceil(timeoutSec / DT);

    double last = Double.NaN;
    for (int i = 0; i < cycles; i++) {
      stepSim();
      sig.refresh();
      last = sig.getValueAsDouble();
      if (Math.abs(last - expected) <= tol) {
        return;
      }
    }
    fail("Duty cycle never reached " + expected + " ± " + tol + " (last=" + last + ")");
  }

  // ---------- Tests ----------

  @Test
  void constructorSucceeds() {
    assertNotNull(shooter);
  }

  @Test
  void dutyCycleOutIsInitialized() {
    assertNotNull(shooter.dutyCycle, "DutyCycleOut should be initialized");
  }

  static Stream<Arguments> dutyCycleCases() {
    return Stream.of(
        Arguments.of(WhichMotor.TOP, 0.75),
        Arguments.of(WhichMotor.BOTTOM, 0.85),
        Arguments.of(WhichMotor.TOP, -0.50),
        Arguments.of(WhichMotor.BOTTOM, -0.30)
    );
  }

  @ParameterizedTest(name = "{0} duty cycle reaches {1}")
  @MethodSource("dutyCycleCases")
  void dutyCycleReachesExpected(WhichMotor which, double power) {
    resetAllMotors();

    var m = motor(which);
    m.setPower().accept(power);

    assertDutyCycleEventually(m, power, DUTY_TOL, DEFAULT_TIMEOUT_S);
  }

  static Stream<Arguments> stopCases() {
    return Stream.of(
        Arguments.of(WhichMotor.TOP, 0.80),
        Arguments.of(WhichMotor.BOTTOM, 0.90)
    );
  }

  @ParameterizedTest(name = "{0} stop brings duty cycle to 0")
  @MethodSource("stopCases")
  void stopBringsDutyCycleToZero(WhichMotor which, double spinPower) {
    var m = motor(which);

    m.setPower().accept(spinPower);
    assertDutyCycleEventually(m, spinPower, DUTY_TOL, DEFAULT_TIMEOUT_S);

    m.stop().run();
    assertDutyCycleEventually(m, 0.0, DUTY_TOL, DEFAULT_TIMEOUT_S);
  }

  @Test
  void bothMotorsAreIndependent() {
    resetAllMotors();

    motor(WhichMotor.TOP).setPower().accept(0.7);
    motor(WhichMotor.BOTTOM).setPower().accept(0.5);

    assertDutyCycleEventually(motor(WhichMotor.TOP), 0.7, DUTY_TOL, DEFAULT_TIMEOUT_S);
    assertDutyCycleEventually(motor(WhichMotor.BOTTOM), 0.5, DUTY_TOL, DEFAULT_TIMEOUT_S);
  }

  @Test
  void realisticShootingSequence() {
    motor(WhichMotor.TOP).setPower().accept(0.9);
    motor(WhichMotor.BOTTOM).setPower().accept(0.85);

    assertDutyCycleEventually(motor(WhichMotor.TOP), 0.9, DUTY_TOL, DEFAULT_TIMEOUT_S);
    assertDutyCycleEventually(motor(WhichMotor.BOTTOM), 0.85, DUTY_TOL, DEFAULT_TIMEOUT_S);

    runForSeconds(3 * DT);

    assertDutyCycleEventually(motor(WhichMotor.TOP), 0.9, DUTY_TOL, DEFAULT_TIMEOUT_S);
    assertDutyCycleEventually(motor(WhichMotor.BOTTOM), 0.85, DUTY_TOL, DEFAULT_TIMEOUT_S);

    resetAllMotors();
    assertDutyCycleEventually(motor(WhichMotor.TOP), 0.0, DUTY_TOL, DEFAULT_TIMEOUT_S);
    assertDutyCycleEventually(motor(WhichMotor.BOTTOM), 0.0, DUTY_TOL, DEFAULT_TIMEOUT_S);
  }

  @Test
  void dutyCycleCanRampThroughMultipleValues() {
    resetAllMotors();

    var m = motor(WhichMotor.TOP);
    for (double power : new double[]{0.3, 0.6, 0.9, 0.0}) {
      m.setPower().accept(power);
      assertDutyCycleEventually(m, power, DUTY_TOL, DEFAULT_TIMEOUT_S);
    }
  }
}