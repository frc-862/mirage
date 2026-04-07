package frc.util.units;

import java.util.Arrays;
import java.util.TreeMap;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

/**
 * Drop-in replacement for {@link ThunderMap} that uses <b>natural cubic spline</b>
 * interpolation instead of linear interpolation.
 * <p>
 * <b>When to use this instead of ThunderMap:</b> whenever the data is non-monotonic
 * or changes slope rapidly between calibration points. Linear interpolation draws
 * straight lines between points, which can produce large errors when the true curve
 * bends. A cubic spline fits smooth curves through every data point, tracking the
 * actual shape much more faithfully.
 * <p>
 * <b>How natural cubic splines work (the short version):</b>
 * <ol>
 *   <li>Between each pair of adjacent data points, we fit a cubic polynomial
 *       (4 unknowns: a + b*x + c*x² + d*x³).</li>
 *   <li>We require that neighboring polynomials agree on value, first derivative,
 *       and second derivative at the shared data point. This gives us a smooth curve
 *       with no kinks.</li>
 *   <li>"Natural" means we set the second derivative to zero at the two endpoints,
 *       which keeps the curve from doing anything wild at the edges.</li>
 *   <li>The system of equations is tridiagonal, so we solve it in O(n) time —
 *       essentially instant for our 7-point maps.</li>
 * </ol>
 * <p>
 * Usage is identical to ThunderMap:
 * <pre>{@code
 * SplineMap<Distance, Time> tofMap = new SplineMap<>() {{
 *     put(Inches.of(60),  Seconds.of(0.88));
 *     put(Inches.of(102), Seconds.of(0.91));
 *     // ...
 * }};
 * Time tof = tofMap.get(Inches.of(150));
 * }</pre>
 *
 * @param <K> Key measure type (e.g. Distance)
 * @param <V> Value measure type (e.g. Time, Angle, AngularVelocity)
 */
public class SplineMap<K extends Measure<?>, V extends Measure<?>> {

    private Unit valueUnit;
    private final TreeMap<Double, Double> rawData = new TreeMap<>();

    // Spline coefficients for each segment [i, i+1).
    // S_i(x) = a[i] + b[i]*(x-x[i]) + c[i]*(x-x[i])^2 + d[i]*(x-x[i])^3
    private double[] xs;
    private double[] a; // = y values
    private double[] b;
    private double[] c;
    private double[] d;
    private boolean built = false;

    public SplineMap() {}

    /**
     * Adds a calibration data point. Units are handled automatically
     * (same as ThunderMap).
     */
    @SuppressWarnings("unchecked")
    public void put(K key, V value) {
        valueUnit = value.baseUnit();
        rawData.put(key.baseUnitMagnitude(), value.baseUnitMagnitude());
        built = false;
    }

    /**
     * Returns the spline-interpolated value at the given key.
     * Outside the data range, the value is clamped to the nearest endpoint
     * (no extrapolation).
     */
    @SuppressWarnings("unchecked")
    public V get(K key) {
        if (!built) {
            buildSpline();
        }
        double x = key.baseUnitMagnitude();
        return (V) valueUnit.of(evaluate(x));
    }

    /**
     * Zero-allocation lookup: takes and returns raw doubles in
     * <b>base SI units</b> (meters, seconds, radians, etc.).
     * Use this in tight loops where GC pressure matters (e.g. the OTF
     * convergence loop on the roboRIO).
     *
     * @param keyBaseUnit key in base units (e.g. meters for Distance)
     * @return interpolated value in base units (e.g. seconds for Time)
     */
    public double getBaseUnit(double keyBaseUnit) {
        if (!built) {
            buildSpline();
        }
        return evaluate(keyBaseUnit);
    }

    // ── Spline construction (runs once, then cached) ──────────────────

    private void buildSpline() {
        int n = rawData.size();
        if (n < 2) {
            throw new IllegalStateException("SplineMap needs at least 2 data points");
        }

        xs = new double[n];
        a  = new double[n];
        int idx = 0;
        for (var entry : rawData.entrySet()) {
            xs[idx] = entry.getKey();
            a[idx]  = entry.getValue();
            idx++;
        }

        // With only 2 points, fall back to linear interpolation
        if (n == 2) {
            b = new double[]{ (a[1] - a[0]) / (xs[1] - xs[0]) };
            c = new double[2];
            d = new double[1];
            built = true;
            return;
        }

        // Step sizes between consecutive data points
        double[] h = new double[n - 1];
        for (int i = 0; i < n - 1; i++) {
            h[i] = xs[i + 1] - xs[i];
        }

        // ── Solve tridiagonal system for c[] (second-derivative / 2) ──
        //
        // For interior points i = 1..n-2, the smoothness condition gives:
        //   h[i-1]*c[i-1] + 2*(h[i-1]+h[i])*c[i] + h[i]*c[i+1]
        //     = 3*( (a[i+1]-a[i])/h[i] - (a[i]-a[i-1])/h[i-1] )
        //
        // Natural boundary: c[0] = c[n-1] = 0.
        //
        // We solve this with the Thomas algorithm (forward sweep + back sub).

        double[] alpha = new double[n];
        for (int i = 1; i < n - 1; i++) {
            alpha[i] = 3.0 / h[i] * (a[i + 1] - a[i])
                      - 3.0 / h[i - 1] * (a[i] - a[i - 1]);
        }

        double[] l  = new double[n];
        double[] mu = new double[n];
        double[] z  = new double[n];

        l[0]  = 1;
        mu[0] = 0;
        z[0]  = 0;

        for (int i = 1; i < n - 1; i++) {
            l[i]  = 2.0 * (xs[i + 1] - xs[i - 1]) - h[i - 1] * mu[i - 1];
            mu[i] = h[i] / l[i];
            z[i]  = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
        }

        c = new double[n];
        b = new double[n - 1];
        d = new double[n - 1];

        // Natural boundary at right endpoint
        c[n - 1] = 0;

        // Back-substitution
        for (int j = n - 2; j >= 0; j--) {
            c[j] = z[j] - mu[j] * c[j + 1];
            b[j] = (a[j + 1] - a[j]) / h[j]
                   - h[j] * (c[j + 1] + 2.0 * c[j]) / 3.0;
            d[j] = (c[j + 1] - c[j]) / (3.0 * h[j]);
        }

        built = true;
    }

    // ── Evaluation ────────────────────────────────────────────────────

    private double evaluate(double x) {
        // Clamp to data range (no extrapolation)
        if (x <= xs[0])              return a[0];
        if (x >= xs[xs.length - 1])  return a[xs.length - 1];

        // Binary search for the segment containing x
        int i = Arrays.binarySearch(xs, x);
        if (i >= 0) return a[i]; // exact hit on a data point

        // binarySearch returns -(insertionPoint) - 1 on miss
        i = -i - 2;

        double dx = x - xs[i];
        return a[i] + dx * (b[i] + dx * (c[i] + dx * d[i]));
    }
}
