package frc.robot.mac;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

public class VisionConstants {
    // A list of tags that we want to ignore in our results
    public static final List<Short> TAG_IGNORE_LIST = List.of();

    // This years field
    public static final AprilTagFieldLayout REBUILT_FIELD = AprilTagFieldLayout
            .loadField(AprilTagFields.k2026RebuiltWelded);

    // Tolerances for result filtering
    public static final double POSE_AMBIGUITY_TOLERANCE = 1;
    public static final double TAG_DISTANCE_TOLERANCE = 6;

    // Carmera constants to store camera name and offsets
    public record CameraConstant(String name, Transform3d offset) {};
    public static final CameraConstant[] CAMERA_CONSTANTS = new CameraConstant[] {
        new CameraConstant("rightCam",
            new Transform3d(
                Inches.of(0),   // forward
                Inches.of(-11.8),   // LEFT
                Inches.of(7.6),    // up
                new Rotation3d(
                    Degrees.of(90), // TODO: fix
                    Degrees.of(-15),  // pitch up
                    Degrees.of(-90)    // yaw outward (left)
                )
            )
        ),
        new CameraConstant("shooterCam",
            new Transform3d(
                Inches.of(12),   // forward
                Inches.of(-10.75), 
                Inches.of(8.5),    // up
                new Rotation3d(
                    Degrees.of(0),
                    Degrees.of(-15),  // pitch up
                    Degrees.of(0)    // yaw outward (left)
                )
            )
        ),
        new CameraConstant("leftCam",
            new Transform3d(
                Inches.of(7.75),   // forward
                Inches.of(12.5),   // LEFT
                Inches.of(8.5),    // up
                new Rotation3d(
                    Degrees.of(0),
                    Degrees.of(-15),  // pitch up
                    Degrees.of(90)    // yaw outward (left)
                )
            )
        )
    };

    public static final CameraConstant[] OASIS_CAMERA_CONSTANTS = new CameraConstant[] {
        new CameraConstant("rightCam",
            new Transform3d(
                Inches.of(0),   // forward
                Inches.of(-11.8),   // LEFT
                Inches.of(7.6),    // up
                new Rotation3d(
                    0,
                    Math.toRadians(-25),  // pitch up
                    Math.toRadians(-90)    // yaw outward (right)
                )
            )
        ),
        new CameraConstant("leftCam", 
            new Transform3d(
                Inches.of(11.25),
                Inches.of(11.25),   // RIGHT side
                Inches.of(10.5),
                new Rotation3d(0, -15, -45)
            )
        ),
    };
}
