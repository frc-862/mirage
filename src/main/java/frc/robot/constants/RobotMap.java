package frc.robot.constants;

import java.nio.file.Paths;

import com.ctre.phoenix6.CANBus;

public class RobotMap {
    public static final String OASIS_IDENTIFIER = "/home/lvuser/Oasis"; // Differentiate between Oasis and Mirage
    public static final boolean IS_OASIS = Paths.get(OASIS_IDENTIFIER).toFile().exists();

    public static final CANBus CAN_BUS = new CANBus("Canivore");

    public static final int COLLECTOR = 9;
    public static final int COLLECTOR_PIVOT = 10;

    public static final int SPINDEXER = 11;
    public static final int TRANSFER = 12;

    public static final int TURRET = 13;
    public static final int TURRET_ENCODER = 35;
    public static final int HOOD = 14;
    public static final int HOOD_ENCODER = 36;
    public static final int SHOOTER_LEFT = 15;
    public static final int SHOOTER_RIGHT = 16;

    public static final int CLIMBER = 17;
    public static final int CLIMBER_ENCODER = 37;

    public static final double CONTROLLER_DEADBAND = 0.1;
    public static final int CONTROLLER_POW = 2;

    public static final int DRIVER_PORT = 0;
    public static final int COPILOT_PORT = 1;
}
