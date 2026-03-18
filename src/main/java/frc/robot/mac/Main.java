package frc.robot.mac;

import java.net.SocketException;

public class Main {
    public static void main(String[] args) {
        // MacMini's constructor now throws SocketException if the UDP socket
        // can't be created. We catch it here and print a clear error message
        // so you know exactly what went wrong, instead of getting a confusing
        // NullPointerException later in the run loop.
        try {
            @SuppressWarnings("resource")
            MacMini mac = new MacMini();
            mac.run();
        } catch (SocketException e) {
            System.err.println("[FATAL] Could not create UDP socket: " + e.getMessage());
            System.err.println("The vision processor cannot run without a network socket.");
            System.exit(1);
        }
    }
}
