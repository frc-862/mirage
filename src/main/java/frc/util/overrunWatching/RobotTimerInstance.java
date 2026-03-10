package frc.util.overrunWatching;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Milliseconds;

import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotController;
import frc.util.dataTypes.Tuple;

public class RobotTimerInstance {
    public class RobotTimer {
        private static RobotTimerInstance instance;

        public static RobotTimerInstance get() {
            return instance;
        }

        public static void startTiming() {
            instance = new RobotTimerInstance();
        }

        public static boolean timing() {
            return instance != null;
        }

        public static Time getTimestamp() {
            return (Milliseconds.of(RobotController.getFPGATime() / 1000.0));
        }

    }
    public record Action(String name) {};
    private enum TimeMeasurementType {
        EXECUTION_TIME("Execution Time"),
        MEAN_EXECUTION_TIME("Mean Execution Time"),
        WEIGHTED_MEAN_EXECUTION_TIME("Weighted Mean Execution Time"),
        MEAN_OVERRUN_EXECUTION_TIME("Mean Overrun Execution Time"),
        MEDIAN_EXECUTION_TIME("Median Execution Time"),
        MEDIAN_OVERRUN_EXECUTION_TIME("Median Overrun Execution Time"),
        MAX_EXECUTION_TIME("Max Execution Time"),
        MAX_OVERRUN_EXECUTION_TIME("Max Overrun Execution Time"),
        PROPORTION_OVERRUN("Proportion of Loops Overrun");

        TimeMeasurementType(String identifier) {
            this.identifier = identifier;
        }

        public String identifier;
    }

    private final ArrayList<Action> actionsTimed;
    private final HashMap<Action, Boolean> lastActiveActions;

    private final HashMap<Tuple<Action, TimeMeasurementType>, DoublePublisher> actionTimePublishers;
    private final BooleanPublisher overrunPublisher;

    private final HashMap<Action, Time> actionExecutionTimes;

    private final HashMap<Action, CRS.MeanTime> meanActionExecutionTimes;
    private final HashMap<Action, CRS.WeightedMeanTime> weightedMeanActionExecutionTimes;
    private final HashMap<Action, CRS.MeanTime> meanOverrideActionExecutionTimes;

    private final HashMap<Action, CRS.MedianTime> medianActionExecutionTimes;
    private final HashMap<Action, CRS.MedianTime> medianOverrideActionExecutionTimes;

    private final HashMap<Action, CRS.MaxTime> maxActionExecutionTimes;
    private final HashMap<Action, CRS.MaxTime> maxOverrideActionExecutionTimes;

    private double loopCount = 0;
    private static final int WARMUP_LOOPS = 50;
    
    private RobotTimerInstance() {
        actionsTimed = new ArrayList<>();
        lastActiveActions = new HashMap<>();

        actionTimePublishers = new HashMap<>();
        overrunPublisher = NetworkTableInstance.getDefault().getTable("Timer").getBooleanTopic("Overrun").publish();

        actionExecutionTimes = new HashMap<>();

        meanActionExecutionTimes = new HashMap<>();
        weightedMeanActionExecutionTimes = new HashMap<>();
        meanOverrideActionExecutionTimes = new HashMap<>();

        medianActionExecutionTimes = new HashMap<>();
        medianOverrideActionExecutionTimes = new HashMap<>();

        maxActionExecutionTimes = new HashMap<>();
        maxOverrideActionExecutionTimes = new HashMap<>();

    }

    public void record(Action action, Time executionTime) {
        // Skip recording during warmup period
        if (loopCount < WARMUP_LOOPS) {
            return;
        }

        if (!actionsTimed.contains(action)) {
            actionsTimed.add(action);

            actionExecutionTimes.put(action, executionTime);

            meanActionExecutionTimes.put(action, new CRS.MeanTime());
            weightedMeanActionExecutionTimes.put(action, new CRS.WeightedMeanTime());
            meanOverrideActionExecutionTimes.put(action, new CRS.MeanTime());

            medianActionExecutionTimes.put(action, new CRS.MedianTime());
            medianOverrideActionExecutionTimes.put(action, new CRS.MedianTime());

            maxActionExecutionTimes.put(action, new CRS.MaxTime());
            maxOverrideActionExecutionTimes.put(action, new CRS.MaxTime());

        }

        lastActiveActions.put(action, true);
        
        actionExecutionTimes.put(action, executionTime);
        publish(action, TimeMeasurementType.EXECUTION_TIME, executionTime);

        Time weightedMeanExecutionTime = weightedMeanActionExecutionTimes.get(action).calculate(executionTime);
        publish(action, TimeMeasurementType.WEIGHTED_MEAN_EXECUTION_TIME, weightedMeanExecutionTime);

        Time meanExecutionTime = meanActionExecutionTimes.get(action).calculate(executionTime);
        publish(action, TimeMeasurementType.MEAN_EXECUTION_TIME, meanExecutionTime);

        Time medianExecutionTime = medianActionExecutionTimes.get(action).calculate(executionTime);
        publish(action, TimeMeasurementType.MEDIAN_EXECUTION_TIME, medianExecutionTime);

        Time maxExecutionTime = maxActionExecutionTimes.get(action).calculate(executionTime);
        publish(action, TimeMeasurementType.MAX_EXECUTION_TIME, maxExecutionTime);
    }

    public void recordCycle(double loopStartTime) {
        loopCount++;
        
        // Skip recording during warmup period
        if (loopCount <= WARMUP_LOOPS) {
            lastActiveActions.replaceAll((action, active) -> false);
            return;
        }
        
        Time cycleTime = RobotTimer.getTimestamp().minus(Microseconds.of(loopStartTime));

        publish(new Action("Main Loop"), TimeMeasurementType.EXECUTION_TIME, cycleTime);

        if (cycleTime.gt(Milliseconds.of(20))) {
            overrunPublisher.accept(true);
            for (Action action : actionsTimed) {
                if (lastActiveActions.get(action)) {
                    Time meanOverrideExecutionTime = meanOverrideActionExecutionTimes.get(action).calculate(actionExecutionTimes.get(action));
                    publish(action, TimeMeasurementType.MEAN_OVERRUN_EXECUTION_TIME, meanOverrideExecutionTime);

                    CRS.MedianTime medianOverrideExecutionTimeCRS = medianOverrideActionExecutionTimes.get(action);
                    Time medianOverrideExecutionTime = medianOverrideExecutionTimeCRS.calculate(actionExecutionTimes.get(action));
                    publish(action, TimeMeasurementType.MEDIAN_OVERRUN_EXECUTION_TIME, medianOverrideExecutionTime);

                    Time maxOverrideExecutionTime = maxOverrideActionExecutionTimes.get(action).calculate(actionExecutionTimes.get(action));
                    publish(action, TimeMeasurementType.MAX_OVERRUN_EXECUTION_TIME, maxOverrideExecutionTime);

                }
            }
        } else {
            overrunPublisher.accept(false);
        }

        lastActiveActions.replaceAll((action, active) -> false);
    }

    private DoublePublisher getPublisher(Action action, TimeMeasurementType measurement) {
        Tuple<Action, TimeMeasurementType> publisherKey = new Tuple<>(action, measurement);

        DoublePublisher publisher = actionTimePublishers.get(publisherKey);

        if (publisher == null) {
            publisher = NetworkTableInstance.getDefault().getTable("Timer").getSubTable(action.name).getDoubleTopic(measurement.identifier).publish();
            actionTimePublishers.put(publisherKey, publisher);
        }

        return publisher;
    }
    
    private void publish(Action action, TimeMeasurementType measurement, Time time) {
        getPublisher(action, measurement).accept(time.in(Milliseconds));
    }

    private void publish(Action action, TimeMeasurementType measurement, double value) {
        getPublisher(action, measurement).accept(value);
    }
}