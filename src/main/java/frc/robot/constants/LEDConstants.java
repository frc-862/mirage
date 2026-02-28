package frc.robot.constants;

import frc.util.leds.LEDStrip;

public class LEDConstants {
    public static final int LED_PWM_PORT = 0;

    public static final int LED_COUNT = 200;

    public static final LEDStrip strip1 = new LEDStrip(10, 0);
    public static final LEDStrip strip2 = new LEDStrip(10, 10);
    public static final LEDStrip strip3 = new LEDStrip(10, 20);
    public static final LEDStrip strip4 = new LEDStrip(10, 30);
    public static final LEDStrip stripAll = new LEDStrip(LED_COUNT, 0);

    public enum LED_STATES {
        TEST,
        VISION_BAD,
        TURRET_BAD,
        SEED_FIELD_FORWARD,
        NEAR_HUB,
        AUTO,
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
