package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.util.AllianceHelpers;

public class FieldConstants {
        public record Target(Translation2d blue, Translation2d red) {}

        public static final Target GOAL_POSITION = new Target(new Translation2d(4.625594, 4.034536), new Translation2d(11.915394, 4.034536));
        public static final Target DEPOT_POSITION = new Target(new Translation2d(0.3937, 0.665988), new Translation2d(16.147288, 7.403338));

        public static Translation2d getTargetData(Target target) {
            return AllianceHelpers.isBlueAlliance() ? target.blue() : target.red();
        }

        // All Rectangle2ds probably have to be changed
        public static final Rectangle2d BLUE_ALLIANCE_RECT = new Rectangle2d(new Pose2d(2.312797, 4.034663, new Rotation2d()), 4.625594, 8.069326); // temp
        public static final Rectangle2d RED_ALLIANCE_RECT = new Rectangle2d(new Pose2d(14.228191, 4.034663, new Rotation2d()), 4.625594, 8.069326); // temp

        public static final Rectangle2d BOTTOM_HALF_RECT = new Rectangle2d(new Pose2d(8.270494, 2.017268, new Rotation2d()), 16.540988, 4.034663); // temp - side on left from perspective of blue driverstation
        public static final Rectangle2d TOP_HALF_RECT = new Rectangle2d(new Pose2d(8.270494, 6.052291, new Rotation2d()), 16.540988, 4.034663); // temp - side on right from perspective of blue driverstation

        public static final Translation2d ZONE_POSITION_BLUE_TOP = new Translation2d(2.034536, 5.963158);
        public static final Translation2d ZONE_POSITION_BLUE_BOTTOM = new Translation2d(2.034536, 2.105914);

        public static final Translation2d ZONE_POSITION_RED_TOP = new Translation2d(13.915394, 5.963158);
        public static final Translation2d ZONE_POSITION_RED_BOTTOM = new Translation2d(13.915394, 2.105914);

        public static final double FIELD_MIDDLE_Y = 4.034663;
    }