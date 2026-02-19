package frc.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.util.units.ThunderMap;
import frc.util.units.ThunderUnits;

class UnitsTest {
    @Test
    void testClampValueBelowMin() {
        Distance value = Meters.of(5);
        Distance min = Meters.of(10);
        Distance max = Meters.of(20);

        var result = ThunderUnits.clamp(value, min, max);

        assertEquals(min, result);
    }

    @Test
    void testClampValueAboveMax() {
        Distance value = Meters.of(25);
        Distance min = Meters.of(10);
        Distance max = Meters.of(20);

        var result = ThunderUnits.clamp(value, min, max);

        assertEquals(max, result);
    }

    @Test
    void testClampValueWithinRange() {
        Distance value = Meters.of(15);
        Distance min = Meters.of(10);
        Distance max = Meters.of(20);

        var result = ThunderUnits.clamp(value, min, max);

        assertEquals(value, result);
    }

    @Test
    void testClampValueEqualsMin() {
        Distance value = Meters.of(10);
        Distance min = Meters.of(10);
        Distance max = Meters.of(20);

        var result = ThunderUnits.clamp(value, min, max);

        assertEquals(value, result);
    }

    @Test
    void testClampValueEqualsMax() {
        Distance value = Meters.of(20);
        Distance min = Meters.of(10);
        Distance max = Meters.of(20);

        var result = ThunderUnits.clamp(value, min, max);

        assertEquals(value, result);
    }

    @Test
    void testClampWithDifferentUnit() {
        LinearVelocity value = MetersPerSecond.of(50);
        LinearVelocity min = MetersPerSecond.of(0);
        LinearVelocity max = MetersPerSecond.of(100);

        var result = ThunderUnits.clamp(value, min, max);

        assertEquals(value, result);
    }

    // Input Modulus Tests

    @Test
    void testModValueBelowMin() {
        Distance value = Meters.of(5);
        Distance min = Meters.of(10);
        Distance max = Meters.of(20);

        var result = ThunderUnits.inputModulus(value, min, max);

        assertEquals(Meters.of(15), result);
    }

    @Test
    void testModValueAboveMax() {
        Distance value = Meters.of(25);
        Distance min = Meters.of(10);
        Distance max = Meters.of(20);

        var result = ThunderUnits.inputModulus(value, min, max);

        assertEquals(Meters.of(15), result);
    }

    @Test
    void testModValueWithinRange() {
        Distance value = Meters.of(15);
        Distance min = Meters.of(10);
        Distance max = Meters.of(20);

        var result = ThunderUnits.inputModulus(value, min, max);

        assertEquals(value, result);
    }

    @Test
    void testModValueEqualsMin() {
        Distance value = Meters.of(10);
        Distance min = Meters.of(10);
        Distance max = Meters.of(20);

        var result = ThunderUnits.inputModulus(value, min, max);

        assertEquals(max, result);
    }

    @Test
    void testModValueEqualsMax() {
        Distance value = Meters.of(20);
        Distance min = Meters.of(10);
        Distance max = Meters.of(20);

        var result = ThunderUnits.inputModulus(value, min, max);

        assertEquals(value, result);
    }

    @Test
    void testModWithDifferentUnit() {
        LinearVelocity value = MetersPerSecond.of(50);
        LinearVelocity min = MetersPerSecond.of(0);
        LinearVelocity max = MetersPerSecond.of(100);

        var result = ThunderUnits.inputModulus(value, min, max);

        assertEquals(value, result);
    }

    @Test
    void testThunderMap() {
        ThunderMap<Distance, LinearVelocity> map = new ThunderMap<>();

        map.put(Meters.of(1), MetersPerSecond.of(1));
        map.put(Meters.of(3), MetersPerSecond.of(3));

        var result = map.get(Meters.of(2));

        LinearVelocity expected = MetersPerSecond.of(2);

        assertEquals(expected, result);
    }

    @Test
    void testThunderMapDiffUnits() {
        ThunderMap<Distance, Angle> map = new ThunderMap<>();

        map.put(Meters.of(1), Rotations.of(1));
        map.put(Inches.of(Units.metersToInches(3)), Degrees.of(Units.rotationsToDegrees(3)));

        var result = map.get(Feet.of(Units.metersToFeet(2)));

        Angle expected = Radians.of(Units.rotationsToRadians(2));
        
        assertEquals(expected, result);
    }
}
