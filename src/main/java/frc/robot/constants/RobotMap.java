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

    // spindexer motor
    public static final int SPINDEXER_MOTOR_ID = 1; // Temp

    // Hood Motor ID
    public static final int HOOD_MOTOR_ID = 3; // Temp

    // Collector / Pivot
    public static final int COLLECTOR_MOTOR_ID = 9; // Temp
    public static final int PIVOT_MOTOR_ID = 5; // Temp
    public static final int PIVOT_ENCODER_ID = 35; // Temp

    //Flywheel
    public static final int FLYWHEEL_MOTOR_ID = 4; //temp
}

