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
        ERROR,
        VISION_BAD,
        NEAR_HUB,
        AUTO,
        SHOOT,
        CANNED_SHOT_START,
        CANNED_SHOT_READY,
        CLIMB,
        COLLECT,
        BRAKE;

        public int id() {
            return this.ordinal();
        }
    }
}
