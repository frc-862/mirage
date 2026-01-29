package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import java.util.stream.Stream;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.util.MotorUnderTest;

public class ShooterTest {
  private static final double BATTERY_VOLTAGE = 12.0;
  private static final double DT = 0.02;          // 20ms
  private static final double DUTY_TOL = 0.01;
  private static final double DEFAULT_TIMEOUT_S = 1.0;

  private Shooter shooter;
  private MotorUnderTest motorUnderTest;

  @BeforeEach
  void setup() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");

    RoboRioSim.setVInVoltage(BATTERY_VOLTAGE);

    var motor = new frc.util.hardware.ThunderBird(
        frc.robot.constants.RobotMap.SHOOTER,
        frc.robot.constants.RobotMap.CAN_BUS,
        frc.robot.constants.ShooterConstants.INVERTED,
        frc.robot.constants.ShooterConstants.STATOR_LIMIT,
        frc.robot.constants.ShooterConstants.BRAKE
    );

    shooter = new Shooter(motor);

    motorUnderTest = new MotorUnderTest(
        motor,
        shooter::setPower,
        shooter::stopMotor,
        motor.getSimState(),
        newKrakenSim()
    );

    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    // CTRE often recommends ~100ms after enable for devices/signals to settle.
    runForSeconds(0.10);
    resetMotor();
  }

  @AfterEach
  void tearDown() throws Exception {
    // Close motor (MotorUnderTest holds ThunderBird)
    if (motorUnderTest != null && motorUnderTest.motor() != null) {
      motorUnderTest.motor().close();
    }

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

  private void stepSim() {
    // If you truly need CTRE async breathing room, keep it *small* and isolated.
    // Otherwise, remove this sleep and see if your tests remain stable.
    try {
      Thread.sleep(5);
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
    }

    motorUnderTest.updateSim(DT, BATTERY_VOLTAGE);
    DriverStationSim.notifyNewData();
  }

  private void runForSeconds(double seconds) {
    int cycles = (int) Math.ceil(seconds / DT);
    for (int i = 0; i < cycles; i++) {
      stepSim();
    }
  }

  private void resetMotor() {
    shooter.setPower(0.0);
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

  static Stream<Arguments> dutyCycleCases() {
    return Stream.of(
        Arguments.of(0.75),
        Arguments.of(0.85),
        Arguments.of(-0.50),
        Arguments.of(-0.30)
    );
  }

  @ParameterizedTest(name = "duty cycle reaches {0}")
  @MethodSource("dutyCycleCases")
  void dutyCycleReachesExpected(double power) {
    resetMotor();

    motorUnderTest.setPower().accept(power);

    assertDutyCycleEventually(motorUnderTest, power, DUTY_TOL, DEFAULT_TIMEOUT_S);
  }

  static Stream<Arguments> stopCases() {
    return Stream.of(
        Arguments.of(0.80),
        Arguments.of(0.90)
    );
  }

  @ParameterizedTest(name = "stop brings duty cycle to 0 (from {0})")
  @MethodSource("stopCases")
  void stopBringsDutyCycleToZero(double spinPower) {
    motorUnderTest.setPower().accept(spinPower);
    assertDutyCycleEventually(motorUnderTest, spinPower, DUTY_TOL, DEFAULT_TIMEOUT_S);

    motorUnderTest.stop().run();
    assertDutyCycleEventually(motorUnderTest, 0.0, DUTY_TOL, DEFAULT_TIMEOUT_S);
  }

  @Test
  void realisticShootingSequence() {
    motorUnderTest.setPower().accept(0.9);

    assertDutyCycleEventually(motorUnderTest, 0.9, DUTY_TOL, DEFAULT_TIMEOUT_S);

    runForSeconds(3 * DT);

    assertDutyCycleEventually(motorUnderTest, 0.9, DUTY_TOL, DEFAULT_TIMEOUT_S);

    resetMotor();
    assertDutyCycleEventually(motorUnderTest, 0.0, DUTY_TOL, DEFAULT_TIMEOUT_S);
  }

  @Test
  void dutyCycleCanRampThroughMultipleValues() {
    resetMotor();

    for (double power : new double[]{0.3, 0.6, 0.9, 0.0}) {
      motorUnderTest.setPower().accept(power);
      assertDutyCycleEventually(motorUnderTest, power, DUTY_TOL, DEFAULT_TIMEOUT_S);
    }
  }
}