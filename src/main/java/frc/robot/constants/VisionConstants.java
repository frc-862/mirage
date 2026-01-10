package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
    // TODO: Update this to be the correect field layout for this season,
    public static final AprilTagFieldLayout TAG_LAYOUT = AprilTagFieldLayout
            .loadField(AprilTagFields.k2025ReefscapeWelded);


    public static enum CameraInfo {

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
}
