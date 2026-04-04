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
        put(Inches.of(102), Seconds.of(0.91));
        put(Inches.of(144), Seconds.of(1.04));
        put(Inches.of(186), Seconds.of(1.17));
        put(Inches.of(228), Seconds.of(1.39));
        put(Inches.of(262), Seconds.of(1.2));
        put(Inches.of(298), Seconds.of(1.4));
    }};

    static ThunderMap<Distance, Time> linearMap = new ThunderMap<>() {{
        put(Inches.of(60),  Seconds.of(0.88));
        put(Inches.of(102), Seconds.of(0.91));
        put(Inches.of(144), Seconds.of(1.04));
        put(Inches.of(186), Seconds.of(1.17));
        put(Inches.of(228), Seconds.of(1.39));
        put(Inches.of(262), Seconds.of(1.2));
        put(Inches.of(298), Seconds.of(1.4));
    }};

    @Test
    void splineHitsExactDataPoints() {
        // Spline must pass through every calibrated value exactly
        assertEquals(0.88, splineMap.get(Inches.of(60)).in(Seconds),  1e-6);
        assertEquals(0.91, splineMap.get(Inches.of(102)).in(Seconds), 1e-6);
        assertEquals(1.04, splineMap.get(Inches.of(144)).in(Seconds), 1e-6);
        assertEquals(1.17, splineMap.get(Inches.of(186)).in(Seconds), 1e-6);
        assertEquals(1.39, splineMap.get(Inches.of(228)).in(Seconds), 1e-6);
        assertEquals(1.2,  splineMap.get(Inches.of(262)).in(Seconds), 1e-6);
        assertEquals(1.4,  splineMap.get(Inches.of(298)).in(Seconds), 1e-6);
    }

    @Test
    void splineClampsOutsideRange() {
        // Below minimum should return the first value
        assertEquals(0.88, splineMap.get(Inches.of(0)).in(Seconds), 1e-6);
        // Above maximum should return the last value
        assertEquals(1.4, splineMap.get(Inches.of(500)).in(Seconds), 1e-6);
    }

    @Test
    void splinePreservesPeakBetterThanLinear() {
        // At 245" (midway in the non-monotonic region 228-262):
        // Linear interpolation draws a straight line from 1.39 → 1.2,
        // giving ~1.295s. The spline should preserve the peak shape better,
        // staying higher than the linear value.
        double linearVal = linearMap.get(Inches.of(245)).in(Seconds);
        double splineVal = splineMap.get(Inches.of(245)).in(Seconds);

        System.out.println("TOF at 245 inches:");
        System.out.println("  Linear: " + linearVal + "s");
        System.out.println("  Spline: " + splineVal + "s");

        // Linear should be about 1.295 (straight line between 1.39 and 1.2)
        assertEquals(1.295, linearVal, 0.01, "Linear interpolation sanity check");

        // Spline should be higher than linear because it follows the curve's
        // upward momentum from 186→228 before bending down toward 262
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

    @Test
    void printConvergenceAnalysis() {
        // Check whether the OTF fixed-point iteration will converge.
        //
        // The iteration is: d_{n+1} = |target - (robot + v * TOF(d_n))|
        // For convergence we need |v * dTOF/dd| < 1 everywhere.
        //
        // We estimate dTOF/dd numerically with a small step (0.1 inch),
        // then multiply by robot speed to get the contraction factor.

        double stepInches = 0.1;
        double[] robotSpeedsMps = {2.0, 3.0, 4.0, 5.0}; // m/s

        System.out.println("\n=== OTF Convergence Analysis (Spline) ===");
        System.out.printf("%-12s", "Dist(in)");
        for (double v : robotSpeedsMps) {
            System.out.printf("| v=%.0fm/s  ", v);
        }
        System.out.println("| dTOF/dd (s/m)");

        System.out.printf("%-12s", "--------");
        for (int i = 0; i < robotSpeedsMps.length; i++) {
            System.out.print("|----------");
        }
        System.out.println("|--------------");

        boolean anyDiverge = false;
        for (int d = 65; d <= 293; d += 5) {
            double tofHi = splineMap.get(Inches.of(d + stepInches)).in(Seconds);
            double tofLo = splineMap.get(Inches.of(d - stepInches)).in(Seconds);
            double dTOF_dd_inchPerSec = (tofHi - tofLo) / (2 * stepInches); // s/inch
            double dTOF_dd_mPerSec = dTOF_dd_inchPerSec * 39.3701; // s/m (39.37 in/m)

            System.out.printf("%-12d", d);
            for (double v : robotSpeedsMps) {
                double contractionFactor = Math.abs(v * dTOF_dd_mPerSec);
                String marker = contractionFactor >= 1.0 ? " **DIV**" : "";
                System.out.printf("| %6.3f%s", contractionFactor, marker);
                if (contractionFactor >= 1.0) anyDiverge = true;
            }
            System.out.printf("| %+.5f%n", dTOF_dd_mPerSec);
        }

        System.out.println("\nContraction factor = |v * dTOF/dd|. Must be < 1 for convergence.");
        if (anyDiverge) {
            System.out.println("WARNING: Some regions show divergence (marked **DIV**)!");
        } else {
            System.out.println("All regions converge at all tested speeds.");
        }

        // Also print the same analysis for linear interpolation
        System.out.println("\n=== OTF Convergence Analysis (Linear) ===");
        System.out.printf("%-12s", "Dist(in)");
        for (double v : robotSpeedsMps) {
            System.out.printf("| v=%.0fm/s  ", v);
        }
        System.out.println("| dTOF/dd (s/m)");

        System.out.printf("%-12s", "--------");
        for (int i = 0; i < robotSpeedsMps.length; i++) {
            System.out.print("|----------");
        }
        System.out.println("|--------------");

        for (int d = 65; d <= 293; d += 5) {
            double tofHi = linearMap.get(Inches.of(d + stepInches)).in(Seconds);
            double tofLo = linearMap.get(Inches.of(d - stepInches)).in(Seconds);
            double dTOF_dd_inchPerSec = (tofHi - tofLo) / (2 * stepInches);
            double dTOF_dd_mPerSec = dTOF_dd_inchPerSec * 39.3701;

            System.out.printf("%-12d", d);
            for (double v : robotSpeedsMps) {
                double contractionFactor = Math.abs(v * dTOF_dd_mPerSec);
                String marker = contractionFactor >= 1.0 ? " **DIV**" : "";
                System.out.printf("| %6.3f%s", contractionFactor, marker);
            }
            System.out.printf("| %+.5f%n", dTOF_dd_mPerSec);
        }
    }

    @Test
    void simulateOTFConvergence() {
        // Simulate the actual OTF iteration for several scenarios:
        //   Robot at various distances, moving toward target at various speeds.
        //
        // For each, run:
        //   1. Spline WITHOUT relaxation (original code behavior)
        //   2. Spline WITH relaxation=0.5 (the fix)
        //   3. Linear WITHOUT relaxation (what the old code did)
        //
        // A "converged" run finds a stable distance. A "diverged" run oscillates.

        double[] startDistancesInches = {150, 200, 240, 270};
        double[] robotSpeedsMps = {3.0, 4.0, 5.0};
        double toleranceMeters = 1.5 * 0.0254; // 1.5 inches in meters
        int maxIter = 15;
        double relaxation = 0.5;

        System.out.println("\n=== OTF Iteration Simulation ===");

        for (double startDist : startDistancesInches) {
            for (double speed : robotSpeedsMps) {
                System.out.printf("\n--- Start: %.0f in, Robot speed: %.1f m/s ---\n", startDist, speed);
                System.out.printf("%-5s | %-22s | %-22s | %-22s%n",
                    "Iter", "Spline (no relax)", "Spline (relax=0.5)", "Linear (no relax)");

                double distSplineRaw = startDist * 0.0254;    // meters
                double distSplineRelax = startDist * 0.0254;
                double distLinear = startDist * 0.0254;

                boolean splineRawConverged = false;
                boolean splineRelaxConverged = false;
                boolean linearConverged = false;

                for (int i = 0; i < maxIter; i++) {
                    // --- Spline, no relaxation ---
                    double newDistSR = distSplineRaw;
                    if (!splineRawConverged) {
                        double tof = splineMap.get(Meters.of(distSplineRaw)).in(Seconds);
                        newDistSR = distSplineRaw - speed * tof; // simplified 1D: moving toward target
                        if (newDistSR < 0) newDistSR = 0.1;     // don't overshoot past target
                        // "converged" = distance didn't change much
                        if (Math.abs(newDistSR - distSplineRaw) * 0.0254 < toleranceMeters) {
                            splineRawConverged = true;
                        }
                    }

                    // --- Spline, with relaxation ---
                    double newDistSX = distSplineRelax;
                    if (!splineRelaxConverged) {
                        double tof = splineMap.get(Meters.of(distSplineRelax)).in(Seconds);
                        double rawDist = distSplineRelax - speed * tof;
                        if (rawDist < 0) rawDist = 0.1;
                        newDistSX = distSplineRelax + relaxation * (rawDist - distSplineRelax);
                        if (Math.abs(newDistSX - distSplineRelax) < toleranceMeters) {
                            splineRelaxConverged = true;
                        }
                    }

                    // --- Linear, no relaxation ---
                    double newDistL = distLinear;
                    if (!linearConverged) {
                        double tof = linearMap.get(Meters.of(distLinear)).in(Seconds);
                        newDistL = distLinear - speed * tof;
                        if (newDistL < 0) newDistL = 0.1;
                        if (Math.abs(newDistL - distLinear) < toleranceMeters) {
                            linearConverged = true;
                        }
                    }

                    System.out.printf("%-5d | %8.4f m %s | %8.4f m %s | %8.4f m %s%n",
                        i,
                        newDistSR, splineRawConverged ? " CONV" : "     ",
                        newDistSX, splineRelaxConverged ? " CONV" : "     ",
                        newDistL, linearConverged ? " CONV" : "     ");

                    distSplineRaw = newDistSR;
                    distSplineRelax = newDistSX;
                    distLinear = newDistL;
                }
            }
        }
    }
}
