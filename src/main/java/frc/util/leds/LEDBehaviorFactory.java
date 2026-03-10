package frc.util.leds;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;

public class LEDBehaviorFactory {

    public static LEDBehavior solid(LEDStrip strip, Color color) {
        return new LEDBehavior((ledController) -> {
            ledController.setStripColor(strip, color);
        });
    }

    /**
     * Creates a rainbow animation.
     *
     * @param strip the LED strip to animate
     * @param speed seconds for 1 full HSV hue rotation (0-180-0)
     * @param color the base color (saturation and value are preserved, hue is cycled)
     * @return the LED behavior
     */
    public static LEDBehavior rainbow(LEDStrip strip, double speed, Color color) {
        return new LEDBehavior((ledController) -> {
            double time = Timer.getFPGATimestamp();
            for (int i = 0; i < strip.length(); i++) {
                int hue = (int)(((i * 180.0 / strip.length()) + (time / speed) * 180.0) % 180.0);
                ledController.setColor(i + strip.startIndex(), color.newHue(hue));
            }
        }).withInterval(speed / 180.0);
    }

    /**
     * Creates a rainbow animation with a default color of RED.
     *
     * @param strip the LED strip to animate
     * @param speed seconds for 1 full HSV hue rotation
     * @return the LED behavior
     */
    public static LEDBehavior rainbow(LEDStrip strip, double speed) {
        return rainbow(strip, speed, Color.RED);
    }

    /**
     * Creates a blink animation.
     *
     * @param strip the LED strip to animate
     * @param speed seconds for 1 full on/off cycle
     * @param color the color to blink
     * @return the LED behavior
     */
    public static LEDBehavior blink(LEDStrip strip, double speed, Color color) {
        return new LEDBehavior((ledController) -> {
            boolean on = (Timer.getFPGATimestamp() % speed) < (speed / 2.0);
            ledController.setStripColor(strip, on ? color : Color.OFF);
        }).withInterval(speed / 2.0);
    }

    /**
     * Creates a pulsing (breathing) animation.
     *
     * @param strip the LED strip to animate
     * @param speed period in seconds for 1 full pulse cycle
     * @param color the color to pulse
     * @return the LED behavior
     */
    public static LEDBehavior pulse(LEDStrip strip, double speed, Color color) {
        return new LEDBehavior((ledController) -> {
            double brightness = 0.5 + 0.5 * Math.sin(2.0 * Math.PI * Timer.getFPGATimestamp() / speed);
            int val = (int)(color.val() * brightness);
            ledController.setStripColor(strip, color.newVal(val));
        }).withInterval(speed / 50.0);
    }

    /**
     * Creates a swirling two-color segment animation.
     *
     * @param strip the LED strip to animate
     * @param speed seconds to move 1 pixel
     * @param segmentSize the size of each color segment in pixels
     * @param color1 the first color
     * @param color2 the second color
     * @return the LED behavior
     */
    public static LEDBehavior swirl(LEDStrip strip, double speed, int segmentSize, Color color1, Color color2) {
        return new LEDBehavior((ledController) -> {
            double pixelOffset = Timer.getFPGATimestamp() / speed;
            for (int i = 0; i < strip.length(); i++) {
                int segmentIndex = ((i + (int) pixelOffset) / segmentSize) % 2;
                ledController.setColor(i + strip.startIndex(), (segmentIndex == 0) ? color1 : color2);
            }
        }).withInterval(speed);
    }

    public static LEDBehavior testStrip(LEDStrip strip, BooleanSupplier... values) {
        if (values.length > strip.length()) {
            throw new IllegalArgumentException("More values for testStrip then on LED strip");
        }
        return new LEDBehavior((ledController) -> {
            for (int i = 0; i < values.length; i++) {
                ledController.setColor(strip.startIndex() + i, (values[i].getAsBoolean() ? Color.GREEN : Color.RED));
            }
        }).dynamic();
    }

    /**
     * Creates a launch (comet/trail) animation.
     *
     * @param strip the LED strip to animate
     * @param speed seconds for 1 full revolution (trail traveling across the entire strip)
     * @param length the length of the lit trail in pixels
     * @param color the color of the trail
     * @return the LED behavior
     */
    public static LEDBehavior launch(LEDStrip strip, double speed, int length, Color color) {
        int totalLength = strip.length() + length;
        return new LEDBehavior((ledController) -> {
            int position = (int)((Timer.getFPGATimestamp() / speed) * totalLength) % totalLength;
            for (int i = 0; i < strip.length(); i++) {
                if (i >= position && i < position + length) {
                    ledController.setColor(i + strip.startIndex(), color);
                } else {
                    ledController.setColor(i + strip.startIndex(), Color.OFF);
                }
            }
        }).withInterval(speed / totalLength);
    }
}
