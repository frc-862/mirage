package frc.robot.constants;

import java.nio.file.Paths;

import com.ctre.phoenix6.CANBus;

public class RobotMap {
    public static final String OASIS_IDENTIFIER = "/home/lvuser/Oasis"; // Differentiate between Oasis and Mirage
    public static final boolean IS_OASIS = Paths.get(OASIS_IDENTIFIER).toFile().exists();

    public static final CANBus CAN_BUS = new CANBus("canivore"); // Temp

    // indexer motor
    public static final int INDEXER_MOTOR_ID = 1; // Temp
    public static final boolean INDEXER_MOTOR_INVERTED = false; // Temp
    public static final double INDEXER_MOTOR_STATOR_LIMIT = 120d; // Temp
    public static final boolean INDEXER_MOTOR_BRAKE_MODE = true; // Temp

    // collector motor
    public static final int COLLECTOR_MOTOR_ID = 2; // Temp
    public static final boolean COLLECTOR_MOTOR_INVERTED = false; // Temp
    public static final double COLLECTOR_MOTOR_STATOR_LIMIT = 120d; // Temp
    public static final boolean COLLECTOR_MOTOR_BRAKE_MODE = true; // Temp

    // collector pivot
    public static final int COLLECTOR_PIVOT_ID = 5; // Temp
    public static final boolean COLLECTOR_PIVOT_INVERTED = false; // Temp
    public static final double COLLECTOR_PIVOT_STATOR_LIMIT = 120d; // Temp
    public static final boolean COLLECTOR_PIVOT_BRAKE_MODE = true; // Temp
}

