package frc.robot.constants;

import java.nio.file.Paths;

import com.ctre.phoenix6.CANBus;

public class RobotMap {
    public static final String OASIS_IDENTIFIER = "/home/lvuser/Oasis"; // Differentiate between Oasis and Mirage
    public static final boolean IS_OASIS = Paths.get(OASIS_IDENTIFIER).toFile().exists();

    public static final CANBus CAN_BUS = new CANBus("Canivore"); // Temp

    // indexer motor
    public static final int INDEXER_MOTOR_ID = 11; // Temp

    // Collector / Pivot
    public static final int COLLECTOR_MOTOR_ID = 2; // Temp
    public static final int PIVOT_MOTOR_ID = 5; // Temp
    public static final int PIVOT_ENCODER_ID = 35; // Temp
}

