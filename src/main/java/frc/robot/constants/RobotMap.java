package frc.robot.constants;

import java.nio.file.Paths;

import com.ctre.phoenix6.CANBus;

public class RobotMap {

    public static final String OASIS_IDENTIFIER = "/home/lvuser/Oasis"; // Differentiate between Oasis and Mirage
    public static final boolean IS_OASIS = Paths.get(OASIS_IDENTIFIER).toFile().exists();

    public static final CANBus CAN_BUS = new CANBus("canbus"); // Temp

    // Shooter constants
    public static final int SHOOTER_BOTTOM_MOTOR_ID = 3; //temp
    public static final int SHOOTER_TOP_MOTOR_ID = 4; //temp
    public static final boolean SHOOTER_BOTTOM_MOTOR_INVERTED = false; //temp
    public static final boolean SHOOTER_TOP_MOTOR_INVERTED = false; //temp
    public static final double SHOOTER_BOTTOM_MOTOR_STATOR_LIMIT = 120.0; //temp
    public static final double SHOOTER_TOP_MOTOR_STATOR_LIMIT = 120.0; //temp
    public static final boolean SHOOTER_BOTTOM_MOTOR_BRAKE = true; //temp
    public static final boolean SHOOTER_TOP_MOTOR_BRAKE = true; //temp

    // Indexer constants
    public static final int INDEXER_MOTOR_ID = 1; // Temp
    public static final boolean INDEXER_MOTOR_INVERTED = false; // Temp
    public static final double INDEXER_MOTOR_STATOR_LIMIT = 120d; // Temp
    public static final boolean INDEXER_MOTOR_BRAKE_MODE = true; // Temp

}

