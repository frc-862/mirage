package frc.robot.constants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class FieldConstants {
    private record Target(Translation2d blue, Translation2d red) {}

    private static final Target GOAL_POSITION = new Target(new Translation2d(4.625594, 4.034536), new Translation2d(11.915394, 4.034536));

    public static Translation2d getTargetData(Target target) {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue ? target.blue() : target.red();
    }
}
