package frc.util.leds;

/*
 * ### Represents a strip of LEDs with a specified length and starting index.
 * 
 * @param length The number of LEDs in the strip.
 * @param startIndex The starting index of the strip in the LED controller.
 */
public record LEDStrip(int length, int startIndex) {
    /*
     * Gets the ending index of the LED strip.
     */
    public int endIndex() {
        return startIndex + length;
    }
}