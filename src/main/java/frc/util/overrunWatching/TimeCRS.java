package frc.util.overrunWatching;

import edu.wpi.first.units.measure.Time;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public abstract class TimeCRS {
    Time time;
    int sampleSize = 1;

    public Time add(Time newTime) {
        if (time == null) {
            time = newTime;
        } else {
            time = calculate(newTime);
        }
        return time;
    }

    public abstract Time calculate(Time time);

    public static class Mean extends TimeCRS {
        public Time calculate(Time newTime) {
            sampleSize++;
            Time deltaTime = newTime.minus(time);
            return time.plus(deltaTime.div(sampleSize));
        }
    }

    public static class WeightedMean extends TimeCRS {
        private double sumOfWeights = 0;

        public Time calculate(Time newTime) {
            sampleSize++;
            
            // Calculate weight based on how close newTime is to the current mean
            // Uses a Gaussian-like weighting: weight = e^(-(deviation/stdDev)^2)
            double newTimeMs = newTime.in(edu.wpi.first.units.Units.Milliseconds);
            double currentMeanMs = time.in(edu.wpi.first.units.Units.Milliseconds);
            
            // Calculate deviation from current mean
            double deviation = Math.abs(newTimeMs - currentMeanMs);
            
            // Standard deviation estimation: use current mean as a scaling factor
            // This allows the weighting to adapt to the scale of the values
            double stdDev = Math.max(currentMeanMs * 0.1, 0.001); // At least 0.001ms
            
            // Calculate weight using Gaussian function
            double weight = Math.exp(-Math.pow(deviation / stdDev, 2));
            
            // Ensure minimum weight to prevent complete rejection of outliers
            weight = Math.max(weight, 0.05);
            
            sumOfWeights += weight;
            
            // Update mean: weighted average update
            double weightedDelta = (newTimeMs - currentMeanMs) * (weight / sumOfWeights);
            Time deltaTime = edu.wpi.first.units.Units.Milliseconds.of(weightedDelta);
            
            return time.plus(deltaTime);
        }
    }

    public static class Median extends TimeCRS {
        private final List<Time> samples = new ArrayList<>();

        @Override
        public Time calculate(Time newTime) {
            samples.add(newTime);
            Collections.sort(samples, (a, b) -> Double.compare(a.in(edu.wpi.first.units.Units.Milliseconds), b.in(edu.wpi.first.units.Units.Milliseconds)));
            int middle = samples.size() / 2;
            if (samples.size() % 2 == 0) {
                return samples.get(middle - 1).plus(samples.get(middle)).div(2);
            } else {
                return samples.get(middle);
            }
        }
    }
}
