package frc.robot.constants;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

public class IndexerConstants {
    // spindexer
    public static final boolean SPINDEXER_MOTOR_INVERTED = false; // temp
    public static final double SPINDEXER_MOTOR_STATOR_LIMIT = 40d; // temp
    public static final boolean SPINDEXER_MOTOR_BRAKE_MODE = true; // temp
    public static final double SPINDEXER_POWER = 1d;

    public static final double SPINDEXER_SIM_kV = 0.24;
    public static final double SPINDEXER_SIM_kA = 0.8;

    // transfer
    public static final boolean TRANSFER_MOTOR_INVERTED = false; // temp
    public static final double TRANSFER_MOTOR_STATOR_LIMIT = 40d; // temp
    public static final boolean TRANSFER_MOTOR_BRAKE_MODE = true; // temp

    // sim
    public static final AngularVelocity SIM_INDEX_THRESHOLD = RotationsPerSecond.of(1); // temp
}
