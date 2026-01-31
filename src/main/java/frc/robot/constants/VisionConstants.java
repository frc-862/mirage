package frc.robot.constants;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import static edu.wpi.first.units.Units.Inches;

public class VisionConstants {
    public static final List<Short> TAG_IGNORE_LIST = List.of();

    // TODO: Update this to be the correect field layout for this season,
    public static final AprilTagFieldLayout DEFAULT_TAG_LAYOUT = AprilTagFieldLayout
            .loadField(AprilTagFields.k2026RebuiltWelded);

    public static final double POSE_AMBIGUITY_TOLERANCE = 1;
    public static final double TAG_DISTANCE_TOLERANCE = 10;

    public record CameraConstant(String name, Transform3d offset) {};

    public static final CameraConstant[] CAMERA_CONSTANTS = new CameraConstant[] {
        new CameraConstant("leftCam",
            new Transform3d(
                Inches.of(11.25),
                Inches.of(-11.25),   // RIGHT side
                Inches.of(10.5),
                new Rotation3d(0, 15, -45))),

        new CameraConstant("rightCam",
            new Transform3d(
                Inches.of(11.25),
                Inches.of(11.25),    // LEFT side
                Inches.of(10.5),
                new Rotation3d(0, 15, 45))),
    };
}
