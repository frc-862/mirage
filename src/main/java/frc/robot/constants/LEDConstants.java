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
        TURRET_LOCKED,
        TURRET_BAD,
        VISION_BAD,
        SEED_FIELD_FORWARD,
        NEAR_HUB,
        CANNED_SHOT_READY,
        HOOD_STOWED,
        SHOOT,
        COLLECT,
        CLIMB;

        public int id() {
            return this.ordinal();
        }
    }
}
