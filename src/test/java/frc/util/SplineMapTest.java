package frc.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import org.junit.jupiter.api.Test;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.util.units.SplineMap;
import frc.util.units.ThunderMap;

/**
 * Tests for SplineMap and OTF convergence analysis.
 *
 * Run with: ./gradlew test --tests "frc.util.SplineMapTest"
 * The print tests produce tables showing interpolation accuracy and convergence.
 */
class SplineMapTest {

    // The actual TOF data from Cannon.java
    static SplineMap<Distance, Time> splineMap = new SplineMap<>() {{
        put(Inches.of(60),  Seconds.of(0.88));
        put(Inches.of(102), Seconds.of(1.0));
        put(Inches.of(144), Seconds.of(1.166));
        put(Inches.of(186), Seconds.of(1.51));
        put(Inches.of(228), Seconds.of(1.4));
        put(Inches.of(262), Seconds.of(1.46));
        put(Inches.of(298), Seconds.of(1.66));
    }};

    static ThunderMap<Distance, Time> linearMap = new ThunderMap<>() {{
        put(Inches.of(60),  Seconds.of(0.88));
        put(Inches.of(102), Seconds.of(1.0));
        put(Inches.of(144), Seconds.of(1.166));
        put(Inches.of(186), Seconds.of(1.51));
        put(Inches.of(228), Seconds.of(1.4));
        put(Inches.of(262), Seconds.of(1.46));
        put(Inches.of(298), Seconds.of(1.66));
    }};

    @Test
    void splineHitsExactDataPoints() {
        // Spline must pass through every calibrated value exactly
        assertEquals(0.88, splineMap.get(Inches.of(60)).in(Seconds),  1e-6);
        assertEquals(1.0, splineMap.get(Inches.of(102)).in(Seconds), 1e-6);
        assertEquals(1.166, splineMap.get(Inches.of(144)).in(Seconds), 1e-6);
        assertEquals(1.51, splineMap.get(Inches.of(186)).in(Seconds), 1e-6);
        assertEquals(1.4, splineMap.get(Inches.of(228)).in(Seconds), 1e-6);
        assertEquals(1.46,  splineMap.get(Inches.of(262)).in(Seconds), 1e-6);
        assertEquals(1.66, splineMap.get(Inches.of(298)).in(Seconds), 1e-6);
    }

    @Test
    void splineClampsOutsideRange() {
        // Below minimum should return the first value
        assertEquals(0.88, splineMap.get(Inches.of(0)).in(Seconds), 1e-6);
        // Above maximum should return the last value
        assertEquals(1.66, splineMap.get(Inches.of(500)).in(Seconds), 1e-6);
    }

    @Test
    void splinePreservesPeakBetterThanLinear() {
        // At 207" (midway between 186 and 228):
        // Linear interpolation draws a straight line from 1.51 -> 1.4,
        // The spline should capture the peak shape better.
        double linearVal = linearMap.get(Inches.of(207)).in(Seconds);
        double splineVal = splineMap.get(Inches.of(207)).in(Seconds);

        System.out.println("TOF at 207 inches:");
        System.out.println("  Linear: " + linearVal + "s");
        System.out.println("  Spline: " + splineVal + "s");

        // Spline should be higher than linear because it follows the curve's
        // upward momentum before bending down
        assertTrue(splineVal > linearVal,
            "Spline should stay higher than linear in the peak region");
    }

    @Test
    void printInterpolationComparison() {
        // Print a comparison table so students can visualize the difference
        System.out.println("\nDistance(in) | Linear TOF(s) | Spline TOF(s) | Diff(s)");
        System.out.println("------------|---------------|---------------|--------");
        for (int d = 60; d <= 298; d += 10) {
            double lin = linearMap.get(Inches.of(d)).in(Seconds);
            double spl = splineMap.get(Inches.of(d)).in(Seconds);
            System.out.printf("%11d | %13.4f | %13.4f | %+.4f%n", d, lin, spl, spl - lin);
        }
    }
}
