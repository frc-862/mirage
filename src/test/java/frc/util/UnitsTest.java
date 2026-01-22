package frc.util;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

class UnitsTest {
    @Test
    void testClampValueBelowMin() {
        Distance value = Meters.of(5);
        Distance min = Meters.of(10);
        Distance max = Meters.of(20);

        var result = Units.clamp(value, min, max);

        assertEquals(min, result);
    }

    @Test
    void testClampValueAboveMax() {
        Distance value = Meters.of(25);
        Distance min = Meters.of(10);
        Distance max = Meters.of(20);

        var result = Units.clamp(value, min, max);

        assertEquals(max, result);
    }

    @Test
    void testClampValueWithinRange() {
        Distance value = Meters.of(15);
        Distance min = Meters.of(10);
        Distance max = Meters.of(20);

        var result = Units.clamp(value, min, max);

        assertEquals(value, result);
    }

    @Test
    void testClampValueEqualsMin() {
        Distance value = Meters.of(10);
        Distance min = Meters.of(10);
        Distance max = Meters.of(20);

        var result = Units.clamp(value, min, max);

        assertEquals(value, result);
    }

    @Test
    void testClampValueEqualsMax() {
        Distance value = Meters.of(20);
        Distance min = Meters.of(10);
        Distance max = Meters.of(20);

        var result = Units.clamp(value, min, max);

        assertEquals(value, result);
    }

    @Test
    void testClampWithDifferentUnit() {
        LinearVelocity value = MetersPerSecond.of(50);
        LinearVelocity min = MetersPerSecond.of(0);
        LinearVelocity max = MetersPerSecond.of(100);

        var result = Units.clamp(value, min, max);

        assertEquals(value, result);
    }
}
