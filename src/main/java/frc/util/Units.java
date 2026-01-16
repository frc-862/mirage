package frc.util;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

public class Units {
  /**
   * Clamps a {@code Measure<U>} between a minimum and maximum.
   *
   * @param value The measure to clamp.
   * @param min The lower bound (inclusive).
   * @param max The upper bound (inclusive).
   * @param <U> The unit type.
   * @return A {@code Measure<U>} between {@code min} and {@code max}.
   */
    public static <U extends Unit> Measure<U> clamp(Measure<U> value, Measure<U> min, Measure<U> max) {
        if (value.lt(min)) {
            return min;
        } else if (value.gt(max)) {
            return max;
        } else {
            return value;
        }
    }
}
