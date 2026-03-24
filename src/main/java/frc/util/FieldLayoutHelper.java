package frc.util;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import com.fasterxml.jackson.databind.node.ObjectNode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;

import java.io.File;
import java.util.Optional;
import java.util.Scanner;

public class FieldLayoutHelper {

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

        double ox = officialPose.get().getX();
        double oy = officialPose.get().getY();
        double oz = officialPose.get().getZ();
        System.out.printf("Official anchor position: (%.4f, %.4f, %.4f)%n", ox, oy, oz);

        // Apply the translation offset to every tag
        for (JsonNode tag : custom.get("tags")) {
            ObjectNode translation = (ObjectNode) tag.get("pose").get("translation");
            translation.put("x", translation.get("x").asDouble() + ox);
            translation.put("y", translation.get("y").asDouble() + oy);
            translation.put("z", translation.get("z").asDouble() + oz);
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
