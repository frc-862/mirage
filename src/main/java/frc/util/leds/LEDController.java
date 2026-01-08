package frc.util.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDController {
    AddressableLED leds;
    AddressableLEDBuffer ledBuffer;

    /*
     * <b>Creates a new LEDController. <br>
     * LEDController is a class for managing LED at a low level.
     * 
     * @param numLEDs The number of LEDs in the LED controller.
     * @param pwmPort The PWM port the LED controller is connected to.
     */
    public LEDController(int numLEDs, int pwmPort) {
        leds = new AddressableLED(pwmPort);
        ledBuffer = new AddressableLEDBuffer(numLEDs);

        leds.setLength(numLEDs);

        leds.start();
    }

    public void apply() {
        leds.setData(ledBuffer);
    }

    public void setColor(int index, int hue, int sat, int val) {
        ledBuffer.setHSV(index, hue, sat, val);
    }

    public void setColor(int index, Color color) {
        ledBuffer.setHSV(index, color.hue(), color.sat(), color.val());
    }

    public void setStripColor(LEDStrip strip, int hue, int sat, int val) {
        for (int i = strip.startIndex(); i < strip.endIndex(); i++) { setColor(i, hue, sat, val); }
    }

    public void setStripColor(LEDStrip strip, Color color) {
        for (int i = strip.startIndex(); i < strip.endIndex(); i++) { setColor(i, color); }
    }

}
