package frc.robot.mac;

import java.nio.file.Paths;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

public class VisionConstants {
    // A list of tags that we want to ignore in our results
    public static final List<Integer> TAG_IGNORE_LIST = List.of(17, 28, 23, 22);

    // This years field
    public static final AprilTagFieldLayout REBUILT_FIELD = AprilTagFieldLayout
            .loadField(AprilTagFields.k2026RebuiltWelded);

    // Tolerances for result filtering
    public static final double POSE_AMBIGUITY_TOLERANCE = 1;
    public static final double TAG_DISTANCE_TOLERANCE = 6;

    public static final boolean IS_OASIS = Paths.get(System.getProperty("user.home"), "Oasis").toFile().exists();

    public static final String LOG_PATH = "/Users/lightning/VisionRunner/vision-logs";

    // Carmera constants to store camera name and offsets
    public record CameraConstant(String name, Transform3d offset) {};
    public static final CameraConstant[] CAMERA_CONSTANTS = new CameraConstant[] {
        new CameraConstant("rightCam",
            new Transform3d(
                Inches.of(0),   // forward
                Inches.of(-11.8),   // LEFT
                Inches.of(7.6),    // up
                new Rotation3d(
                    Degrees.of(0),
                    Degrees.of(340), // pitch up
                    Degrees.of(270)   // y/ yaw outward (left)
                )
            )
        ),
        new CameraConstant("shooterCam",
            new Transform3d(
                Inches.of(11.5),   // forward
                Inches.of(-11.2), 
                Inches.of(8.75),    // up
                new Rotation3d(
                    Degrees.of(0),
                    Degrees.of(340), // pitch up
                    Degrees.of(0)   // yaw outward (left)
                )
            )
        ),
        new CameraConstant("leftCam",
            new Transform3d(
                Inches.of(7.1),   // forward
                Inches.of(12.3),   // LEFT
                Inches.of(8.5),    // up
                new Rotation3d(
                    Degrees.of(0),
                    Degrees.of(340), // pitch up
                    Degrees.of(90)   // y
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
