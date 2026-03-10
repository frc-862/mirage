package frc.util.overrunWatching;

import edu.wpi.first.units.measure.Time;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public abstract class CRS {

    public static class MeanTime extends CRS {
        private Time mean = null;
        private int sampleSize = 0;

        public Time calculate(Time newTime) {
            if (mean == null) {
                mean = newTime;
                return mean;
            }
            sampleSize++;
            Time deltaTime = newTime.minus(mean);
            return mean.plus(deltaTime.div(sampleSize));
        }

        public Time getMean() {
            return mean;
        }

        public int getSampleSize() {
            return sampleSize;
        }
    }

    public static class WeightedMeanTime extends CRS {
        private Time weightedMean = null;
        private double sumOfWeights = 0;

        public Time calculate(Time newTime) {
            if (weightedMean == null) {
                weightedMean = newTime;
                return weightedMean;
            }
            // Calculate weight based on how close newTime is to the current mean
            // Uses a Gaussian-like weighting: weight = e^(-(deviation/stdDev)^2)
            double newTimeMs = newTime.in(edu.wpi.first.units.Units.Milliseconds);
            double currentMeanMs = weightedMean.in(edu.wpi.first.units.Units.Milliseconds);
            
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
            
            return weightedMean.plus(deltaTime);
        }

        public Time getWeightedMean() {
            return weightedMean;
        }

        public double getSumOfWeights() {
            return sumOfWeights;
        }
    }

    public static class MedianTime extends CRS {
        private final List<Time> samples = new ArrayList<>();
        private Time median = null;

        public Time calculate(Time newTime) {
            samples.add(newTime);
            Collections.sort(samples, (a, b) -> Double.compare(a.in(edu.wpi.first.units.Units.Milliseconds), b.in(edu.wpi.first.units.Units.Milliseconds)));
            int middle = samples.size() / 2;
            if (samples.size() % 2 == 0) {
                median = samples.get(middle - 1).plus(samples.get(middle)).div(2);
            } else {
                median = samples.get(middle);
            }
            return median;
        }

        public Time getTime() {
            return median;
        }

        public int getSampleSize() {
            return samples.size();
        }
        
    }

    public static class MaxTime extends CRS {
        private Time max = null;

        public Time calculate(Time newTime) {
            if (max == null || newTime.gt(max)) {
                max = newTime;
                return newTime;
            } else {
                return max;
            }
        }
    }

    public static class Proportion {
        private double total = 0;
        private double count = 0;

        public double calculate(boolean condition) {
            total++;
            if (condition) {
                count++;
            }
            return count / total;
        }
    }
}
