package frc.util.leds;

import java.util.ArrayList;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.Timer;

public class LEDBehavior {
    private final ArrayList<Consumer<LEDController>> behaviors;
    private double updateInterval;
    private double lastUpdateTime;
    private boolean isDynamic;

    public LEDBehavior() {
        behaviors = new java.util.ArrayList<>();
        updateInterval = 0.0;
        lastUpdateTime = 0.0;
        isDynamic = false;
    }

    public LEDBehavior(Consumer<LEDController> behavior) {
        this();

        behaviors.add(behavior);
    }

    public LEDBehavior and(Consumer<LEDController> behavior) {
        behaviors.add(behavior);

        return this;
    }

    public LEDBehavior and(LEDBehavior other) {
        behaviors.addAll(other.behaviors);

        return this;
    }

    /**
     * Checks if enough time has passed since the last update for this behavior to run again.
     * Static behaviors (interval == 0, not dynamic) always return true (they only need to be set once).
     * Dynamic behaviors (interval < 0) always return true (they update every cycle).
     * Timed behaviors return true once the interval has elapsed.
     *
     * @return true if this behavior should be applied this cycle
     */
    public boolean shouldUpdate() {
        if (!isDynamic && updateInterval == 0.0) {
            return true; // static behavior, always apply (set-and-forget)
        }

        double now = Timer.getFPGATimestamp();
        if (updateInterval < 0 || (now - lastUpdateTime) >= updateInterval) {
            lastUpdateTime = now;
            return true;
        }
        return false;
    }

    public void apply(LEDController leds) {
        behaviors.forEach(behavior -> behavior.accept(leds));
    }

    /**
     * Sets the update interval in seconds between LED refreshes for this behavior.
     *
     * @param interval the interval in seconds. Use a negative value for every-cycle updates.
     * @return this behavior for chaining
     */
    public LEDBehavior withInterval(double interval) {
        updateInterval = interval;
        isDynamic = true;
        return this;
    }

    /**
     * Marks this behavior as dynamic (updates every cycle).
     *
     * @return this behavior for chaining
     */
    public LEDBehavior dynamic() {
        updateInterval = -1;
        isDynamic = true;
        return this;
    }

    /**
     * Gets the update interval for this behavior.
     *
     * @return the update interval in seconds
     */
    public double getUpdateInterval() {
        return updateInterval;
    }

}
