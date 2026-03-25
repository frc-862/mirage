package frc.util;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import com.fasterxml.jackson.databind.node.ObjectNode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

import java.io.File;
import java.util.Optional;
import java.util.Scanner;

// 100% vibecoded

public class FieldLayoutHelper {

    private static Rotation3d quaternionToRotation(JsonNode q) {
        return new Rotation3d(
            new edu.wpi.first.math.geometry.Quaternion(
                q.get("W").asDouble(),
                q.get("X").asDouble(),
                q.get("Y").asDouble(),
                q.get("Z").asDouble()
            )
        );
    }

    private static void writeQuaternion(ObjectNode rotNode, Rotation3d rot) {
        var q = rot.getQuaternion();
        ObjectNode qNode = (ObjectNode) rotNode.get("quaternion");
        qNode.put("W", q.getW());
        qNode.put("X", q.getX());
        qNode.put("Y", q.getY());
        qNode.put("Z", q.getZ());
    }

    public static void main(String[] args) throws Exception {
        Scanner scanner = new Scanner(System.in);
        ObjectMapper mapper = new ObjectMapper().enable(SerializationFeature.INDENT_OUTPUT);

        String home = System.getProperty("user.home");

        System.out.print("Field layout filename (relative to ~): ");
        String filename = scanner.nextLine().trim().replace("'", "").replace("\"", "");

        File customFile = new File(home, filename);
        System.out.println("Loading: " + customFile.getAbsolutePath());

        JsonNode custom = mapper.readTree(customFile);

        AprilTagFieldLayout officialLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        // Find the anchor tag at (0, 0, 0) in the custom layout
        JsonNode anchorTag = null;
        for (JsonNode tag : custom.get("tags")) {
            JsonNode t = tag.get("pose").get("translation");
            if (t.get("x").asDouble() == 0.0
                    && t.get("y").asDouble() == 0.0
                    && t.get("z").asDouble() == 0.0) {
                anchorTag = tag;
                break;
            }
        }

        if (anchorTag == null) {
            System.err.println("Error: No tag found at position (0, 0, 0) in your layout.");
            System.exit(1);
        }

        int anchorId = anchorTag.get("ID").asInt();
        System.out.println("Anchor tag: ID " + anchorId);

        Optional<Pose3d> officialPose = officialLayout.getTagPose(anchorId);
        if (officialPose.isEmpty()) {
            System.err.println("Error: Tag ID " + anchorId + " not found in official 2026 Rebuilt Welded layout.");
            System.exit(1);
        }

        Translation3d officialTranslation = officialPose.get().getTranslation();
        Rotation3d officialRotation = officialPose.get().getRotation();
        System.out.printf("Official anchor position: (%.4f, %.4f, %.4f)%n",
                officialTranslation.getX(), officialTranslation.getY(), officialTranslation.getZ());

        // Get the anchor's rotation from the custom layout
        Rotation3d customAnchorRotation = quaternionToRotation(
                anchorTag.get("pose").get("rotation").get("quaternion"));

        // R_correction = R_official * R_custom^(-1), maps custom frame -> field frame
        Rotation3d rotationCorrection = customAnchorRotation.unaryMinus().plus(officialRotation);

        // Transform every tag: rotate its position into field frame, then translate
        for (JsonNode tag : custom.get("tags")) {
            ObjectNode translation = (ObjectNode) tag.get("pose").get("translation");
            double x = translation.get("x").asDouble();
            double y = translation.get("y").asDouble();
            double z = translation.get("z").asDouble();

            Translation3d rotated = new Translation3d(x, y, z).rotateBy(rotationCorrection);

            translation.put("x", rotated.getX() + officialTranslation.getX());
            translation.put("y", rotated.getY() + officialTranslation.getY());
            translation.put("z", rotated.getZ() + officialTranslation.getZ());

            // Also correct each tag's own rotation
            ObjectNode rotNode = (ObjectNode) tag.get("pose").get("rotation");
            Rotation3d customTagRotation = quaternionToRotation(rotNode.get("quaternion"));
            Rotation3d correctedRotation = customTagRotation.plus(rotationCorrection);
            writeQuaternion(rotNode, correctedRotation);
        }

        // Use official field dimensions
        ObjectNode fieldNode = mapper.createObjectNode();
        fieldNode.put("length", officialLayout.getFieldLength());
        fieldNode.put("width", officialLayout.getFieldWidth());
        ((ObjectNode) custom).set("field", fieldNode);

        // Bounds check -- warn about any tags outside the field
        double fieldLength = officialLayout.getFieldLength();
        double fieldWidth = officialLayout.getFieldWidth();
        boolean anyOutOfBounds = false;

        for (JsonNode tag : custom.get("tags")) {
            int id = tag.get("ID").asInt();
            JsonNode t = tag.get("pose").get("translation");
            double x = t.get("x").asDouble();
            double y = t.get("y").asDouble();

            if (x < 0 || x > fieldLength || y < 0 || y > fieldWidth) {
                System.err.printf("WARNING: Tag %d is off the field! (x=%.4f, y=%.4f) "
                        + "[field is %.2f x %.2f]%n", id, x, y, fieldLength, fieldWidth);
                anyOutOfBounds = true;
            }
        }

        if (anyOutOfBounds) {
            System.err.println("\nSome tags are outside field bounds. Double-check your measurements.");
        }

        String output = mapper.writeValueAsString(custom);

        System.out.println("\nCorrected " + custom.get("tags").size() + " tags. Copy the JSON below:\n");
        System.out.println(output);

        scanner.close();
    }
}
