package frc.robot.constants;

import java.nio.file.Paths;

import com.ctre.phoenix6.CANBus;

public class RobotMap {
    public static final String OASIS_IDENTIFIER = "/home/lvuser/Oasis"; // Differentiate between Oasis and Mirage
    public static final boolean IS_OASIS = Paths.get(OASIS_IDENTIFIER).toFile().exists();

    public static final CANBus CAN_BUS = new CANBus("Canivore");

    public static final int TURRET_MOTOR_ID = 0; // Temp
    public static final int TURRET_ENCODER_ID = 0; // Temp
    public static final boolean TURRET_MOTOR_INVERTED = false; // Temp
    public static final int TURRET_STATOR_LIMIT = 0; // Temp
    public static final boolean TURRET_BRAKE = false; // Temp

    // indexer motor
    public static final int INDEXER_MOTOR_ID = 1; // Temp
    public static final boolean INDEXER_MOTOR_INVERTED = false; // Temp
    public static final double INDEXER_MOTOR_STATOR_LIMIT = 120d; // Temp
    public static final boolean INDEXER_MOTOR_BRAKE_MODE = true; // Temp

    // Collector / Pivot
    public static final int COLLECTOR_MOTOR_ID = 2; // Temp
    public static final int PIVOT_MOTOR_ID = 5; // Temp
    public static final int PIVOT_ENCODER_ID = 35; // Temp
}

