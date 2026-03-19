package frc.robot.mac;

public class Main {
    // How long to wait before retrying after an error (milliseconds).
    // 2 seconds is long enough to avoid hammering a broken resource
    // (e.g., a port in use) but short enough that the vision processor
    // recovers quickly when the problem resolves.
    private static final long RETRY_DELAY_MS = 2000;

    public static void main(String[] args) {
        // ===== RETRY LOOP =====
        // This is the core of our error recovery strategy on the Mac side.
        //
        // WHY: Many things can go wrong during a match:
        //   - The network cable gets bumped → socket dies
        //   - PhotonVision crashes and restarts → NetworkTables disconnects
        //   - The RIO reboots → temporary network loss
        //   - An unexpected exception we didn't anticipate
        //
        // OLD BEHAVIOR: Any error was fatal — the vision processor died
        // and stayed dead for the rest of the match. Someone had to SSH
        // in and manually restart it.
        //
        // NEW BEHAVIOR: If anything goes wrong, we:
        //   1. Clean up the old MacMini (close socket, NT connection, etc.)
        //   2. Wait 2 seconds (so we don't spin if the error keeps happening)
        //   3. Create a fresh MacMini and try again
        //
        // This means the vision processor is self-healing. Even if it
        // crashes 5 times during a match, it'll keep coming back.
        while (true) {
            try (MacMini mac = new MacMini()) {
                System.out.println("[PHOTON VISION] Started successfully, entering run loop");
                mac.run();
                // run() only returns when a fatal error occurs (e.g., socket died).
                // Fall through to the retry logic below.
                System.out.println("[PHOTON VISION] run() exited, will retry...");

            } catch (Exception e) {
                // This catches both IOException (socket/network problems) and
                // any unexpected RuntimeException. We log the error and retry.
                System.err.println("[PHOTON VISION] Error: " + e.getMessage());
                e.printStackTrace();
            }

            // Wait before retrying. This prevents us from spinning in a
            // tight loop if the error happens immediately on startup
            // (e.g., port already in use, network interface down).
            System.out.println("[PHOTON VISION] Retrying in " + RETRY_DELAY_MS + "ms...");
            try {
                Thread.sleep(RETRY_DELAY_MS);
            } catch (InterruptedException e) {
                // If someone interrupts our sleep, just retry immediately
                Thread.currentThread().interrupt();
            }
        }
    }
}
