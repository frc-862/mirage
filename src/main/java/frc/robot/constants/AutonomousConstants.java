package frc.robot.constants; // temp

import static edu.wpi.first.units.Units.Amps; // temp
import static edu.wpi.first.units.Units.KilogramSquareMeters; // temp
import static edu.wpi.first.units.Units.Pounds; // temp

import com.pathplanner.lib.config.ModuleConfig; // temp
import com.pathplanner.lib.config.PIDConstants; // temp
import com.pathplanner.lib.config.RobotConfig; // temp

import edu.wpi.first.math.geometry.Translation2d; // temp
import edu.wpi.first.math.system.plant.DCMotor; // temp
import edu.wpi.first.units.measure.Mass; // temp
import edu.wpi.first.units.measure.MomentOfInertia; // temp

public class AutonomousConstants {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(50, 0, 0); // TODO: Tune
    public static final PIDConstants ROTATION_PID = new PIDConstants(5, 0, 0); // temp

    protected static final Mass ROBOT_MASS = Pounds.of(88); // TODO: Update
    private static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(3.3927854218); // temp

    private static final ModuleConfig MODULE_CONFIG = new ModuleConfig(
        DriveConstants.kWheelRadius, DriveConstants.kSpeedAt12Volts,
        DriveConstants.COF, DCMotor.getKrakenX60Foc(1).withReduction(DriveConstants.kDriveGearRatio),
        Amps.of(120), 1);

    public static final RobotConfig getConfig(Translation2d... moduleLocations) {
        return new RobotConfig(ROBOT_MASS, ROBOT_MOI, MODULE_CONFIG, moduleLocations);
    }
}
