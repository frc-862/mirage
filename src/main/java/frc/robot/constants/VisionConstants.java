package frc.robot.constants;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
    public enum CameraInfo {

        FRONT_RIGHT("frontRight", new Transform3d()), 
        BACK_RIGHT("backRight", new Transform3d()), 
        FRONT_LEFT("frontLeft", new Transform3d()),
        BACK_LEFT("backLeft", new Transform3d());

        public Transform3d offset;
        public String name;

        CameraInfo(String name, Transform3d offset) {
            this.offset = offset;
            this.name = name;
        }
    }

    public static final List<Short> TAG_IGNORE_LIST = List.of();

    // TODO: Update this to be the correect field layout for this season,
    public static final AprilTagFieldLayout TAG_LAYOUT = AprilTagFieldLayout
            .loadField(AprilTagFields.kDefaultField);

    public static final double POSE_AMBIGUITY_TOLERANCE = 0.2;
    public static final double TAG_DISTANCE_TOLERANCE = 4;
}
