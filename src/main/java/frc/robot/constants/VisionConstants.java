package frc.robot.constants;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {

    public static final List<Short> TAG_IGNORE_LIST = List.of();

    public static final AprilTagFieldLayout TAG_LAYOUT = AprilTagFieldLayout
            .loadField(AprilTagFields.k2026RebuiltWelded); //FIM Uses welded fields

    public static final double POSE_AMBIGUITY_TOLERANCE = 0.2;
    public static final double TAG_DISTANCE_TOLERANCE = 4;

    public record CameraConstant(String name, Transform3d offset) {};

    public static final CameraConstant[] CAMERA_CONSTANTS = new CameraConstant[] {
        new CameraConstant("frontRight", new Transform3d()),
        new CameraConstant("backRight", new Transform3d()),
        new CameraConstant("frontLeft", new Transform3d()),
        new CameraConstant("backLeft", new Transform3d())
    };
}
