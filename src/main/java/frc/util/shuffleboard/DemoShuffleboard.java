package frc.util.shuffleboard;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;

public class DemoShuffleboard {
    public static DoubleSubscriber subscribeToDouble(String key, double defaultValue) {
        NetworkTableInstance.getDefault().getTable("Demo").getEntry(key).setDouble(defaultValue);

        return NetworkTableInstance.getDefault().getTable("Demo")
            .getDoubleTopic(key).subscribe(defaultValue);
    }

     public static BooleanSubscriber subscribeToBoolean(String key, boolean defaultValue) {
        NetworkTableInstance.getDefault().getTable("Demo").getEntry(key).setBoolean(defaultValue);

        return NetworkTableInstance.getDefault().getTable("Demo")
            .getBooleanTopic(key).subscribe(defaultValue);
    }

}
