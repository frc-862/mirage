package frc.util.units;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

public class ThunderUnits {
  /**
   * Clamps a {@code Measure<U>} between a minimum and maximum.
   *
   * @param value The measure to clamp.
   * @param min The lower bound (inclusive).
   * @param max The upper bound (inclusive).
   * @param <U> The unit type.
   * @param <M> The measure type.
   * @return A {@code Measure<U>} between {@code min} and {@code max}.
   */
    public static <U extends Unit, M extends Measure<U>> M clamp(M value, M min, M max) {
        if (value.lt(min)) {
            return min;
        } else if (value.gt(max)) {
            return max;
        } else {
            return value;
        }
    }

    /**
     * Wraps a {@code Measure<U>} between a minimum and maximum.
     *
     * @param value
     * @param min the lower bound (exclusive).
     * @param max the upper bound (inclusive).
     * @param <U> The unit type.
     * @param <M> The measure type.
     * @return A {@code Measure<U>} between {@code min} and {@code max}.
     */
    @SuppressWarnings("unchecked")
    public static <U extends Unit, M extends Measure<U>> M inputModulus(M value, M min, M max) {
        Measure<U> modulus = max.minus(min);

        // Wrap input if it's above the maximum input
        int numMax = (int) value.minus(min).div(modulus).magnitude();
        value.minus(modulus.times(numMax));
        value = (M) value.minus(modulus.times(numMax));

        // Wrap input if it's below the minimum input
        int numMin = (int) value.minus(max).div(modulus).magnitude();
        value = (M) value.minus(modulus.times(numMin));

        return value;
    }
}