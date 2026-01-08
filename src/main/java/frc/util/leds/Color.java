package frc.util.leds;

public record Color(int hue, int sat, int val) {
    public static final Color RED = new Color(0, 255, 255);
    public static final Color ORANGE = new Color(5, 255, 255);
    public static final Color YELLOW = new Color(15, 255, 255);
    public static final Color GREEN = new Color(60, 255, 255);
    public static final Color BLUE = new Color(120, 255, 255);
    public static final Color PURPLE = new Color(140, 255, 255);
    public static final Color PINK = new Color(165, 255, 255);
    public static final Color WHITE = new Color(0, 0, 255);
    public static final Color GREY = new Color(0, 0, 128);

    public static final Color OFF = new Color(0, 0, 0);

    public Color newHue(int hue) {
        return new Color(hue, this.sat, this.val);
    }

    public Color newSat(int sat) {
        return new Color(this.hue, sat, this.val);
    }

    public Color newVal(int val) {
        return new Color(this.hue, this.sat, val);
    }

}
