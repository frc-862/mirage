package frc.util.leds;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;

public class LEDBehaviorFactory {

    public static LEDBehavior SolidColorBehavior(LEDStrip strip, Color color) {
        return new LEDBehavior((ledController) -> {
            ledController.setStripColor(strip, color);
        });
    }

    public static LEDBehavior RainbowBehavior(LEDStrip strip, double speed, Color color) {
        return new LEDBehavior((ledController) -> {
            for (int i = 0; i < strip.length(); i++) {
                int hue = (int)(((i * 180.0 / strip.length()) + (int)((Timer.getFPGATimestamp() * 180.0) / speed)) % 180.0);
                ledController.setColor(i + strip.startIndex(), color.newHue(hue));
            }
        });
    }

    public static LEDBehavior RainbowBehavior(LEDStrip strip, int speed) {
        return RainbowBehavior(strip, speed, Color.RED);
    }
 
    public static LEDBehavior BlinkColorBehavior(LEDStrip strip, double speed, Color color) {
        return new LEDBehavior((ledController) -> {
            int switchValue = ((int)(Timer.getFPGATimestamp() * speed)) % 2;
            ledController.setStripColor(strip, switchValue == 0 ? color : Color.OFF);
        });
    }
    
    public static LEDBehavior pulseColorBehavior(LEDStrip strip, double speed, Color color) {
        return new LEDBehavior((ledController) -> {
            int val = (int)(color.val() * (0.5 + 0.5 * Math.sin(Timer.getFPGATimestamp() * speed * 2 * Math.PI)));
            ledController.setStripColor(strip, color.newVal(val));
        });
    }

    public static LEDBehavior SwirlBehabior(LEDStrip strip, double speed, int segmentSize, Color color1, Color color2) {
        return new LEDBehavior((ledController) -> {
            for (int i = 0; i < strip.length(); i++) {
                int segmentIndex = ((i + (int)(Timer.getFPGATimestamp() * speed)) / segmentSize) % 2;
                ledController.setColor(i + strip.startIndex(), (segmentIndex == 0) ? color1 : color2);
            }
        });
    }

    public static LEDBehavior TestStripBehavior(int startIndex, BooleanSupplier... values) {
        return new LEDBehavior((ledController) -> {
            for (int i = 0; i < values.length; i++) {
                ledController.setColor(startIndex + i, (values[i].getAsBoolean() ? Color.GREEN : Color.RED));
            }
        });
    }

    public static LEDBehavior LaunchBehavior(LEDStrip strip, double speed, int length, Color color) {
        return new LEDBehavior((ledController) -> {
            int position = (int)(Timer.getFPGATimestamp() * speed) % (strip.length() + length);
            for (int i = 0; i < strip.length(); i++) {
                if (i >= position && i < position + length) {
                    ledController.setColor(i + strip.startIndex(), color);
                } else {
                    ledController.setColor(i + strip.startIndex(), Color.OFF);
                }
            }
        });
    }
}
