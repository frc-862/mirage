package frc.robot.constants;

import frc.util.leds.LEDStrip;

public class LEDConstants {
    public static final int LED_PWM_PORT = 0;

    public static final int LED_COUNT = 300;

    public static final LEDStrip stripUnderglow = new LEDStrip(56, 0);
    public static final LEDStrip stripShooter = new LEDStrip(115, 56);
    public static final LEDStrip stripAll = new LEDStrip(LED_COUNT, 0);

    public enum LED_STATES {
        TEST,
        TURRET_MANUAL,
        TURRET_BAD,
        VISION_BAD,
        // VISION_COMMS_NO_POSE: Mac Mini is alive and sending heartbeats,
        // but no AprilTags are visible. Shows blinking yellow on the
        // underglow while disabled so the driver knows "comms good, just
        // no tags" vs "everything is broken" (VISION_BAD = solid red).
        VISION_COMMS_NO_POSE,
        SEED_FIELD_FORWARD,
        NEAR_HUB,
        AUTO,
        CANNED_SHOT_READY,
        SHOOT,
        COLLECT,
        CLIMB,
        HOOD_STOWED;

        public int id() {
            return this.ordinal();
        }
    }
}
