package frc.robot.mac;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import static edu.wpi.first.units.Units.Inches;

public class VisionConstants {
    // A list of tags that we want to ignore in our results
    public static final List<Short> TAG_IGNORE_LIST = List.of();

    // This years field
    public static final AprilTagFieldLayout REBUILT_FIELD = AprilTagFieldLayout
            .loadField(AprilTagFields.k2026RebuiltWelded);

    // Tolerances for result filtering
    public static final double POSE_AMBIGUITY_TOLERANCE = 1;
    public static final double TAG_DISTANCE_TOLERANCE = 10;

    // Carmera constants to store camera name and offsets
    public record CameraConstant(String name, Transform3d offset) {};
    public static final CameraConstant[] CAMERA_CONSTANTS = new CameraConstant[] {
        new CameraConstant("leftCam",
            new Transform3d(
                Inches.of(11.25),   // forward
                Inches.of(11.25),   // LEFT
                Inches.of(10.5),    // up
                new Rotation3d(
                    0.0,
                    Math.toRadians(-15),  // pitch up
                    Math.toRadians(45)    // yaw outward (left)
                )
            )
        ),

        new CameraConstant("rightCam",
            new Transform3d(
                Inches.of(11.25),   // forward
                Inches.of(-11.25),  // RIGHT
                Inches.of(10.5),    // up
                new Rotation3d(
                    0.0,
                    Math.toRadians(-15),   // pitch up
                    Math.toRadians(-45)    // yaw outward (right)
                )
            )
        ),
    };

}
