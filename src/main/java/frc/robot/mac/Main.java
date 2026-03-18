package frc.robot.mac;

import java.io.IOException;

public class Main {
    public static void main(String[] args) {
        // MacMini's constructor throws IOException if the UDP socket can't be
        // created or the RoboRIO address can't be resolved. We catch it here
        // and print a clear error message so you know exactly what went wrong,
        // instead of getting a confusing NullPointerException later in the run loop.
        try {
            @SuppressWarnings("resource")
            MacMini mac = new MacMini();
            mac.run();
        } catch (IOException e) {
            System.err.println("[FATAL] Could not initialize networking: " + e.getMessage());
            System.err.println("The vision processor cannot run without a network socket.");
            System.exit(1);
        }
    }
}
