package frc.util.leds;

import java.util.function.BooleanSupplier;

public class LEDLatch implements BooleanSupplier {
    private final BooleanSupplier condition;
    private boolean latchedState = false;

    /**
     * Creates a new LEDLatch.
     * 
     * @param condition - condition to latch
     */
    public LEDLatch(BooleanSupplier condition) {
        this.condition = condition;
    }

    /**
     * Returns true if the condition has ever been true since the last reset.
     * 
     * @return boolean - latched state
     */
    @Override
    public boolean getAsBoolean() {
        if (condition.getAsBoolean()) {
            latchedState = true;
        }
        return latchedState;
    }

    /**
     * Resets the latched state to false.
     */
    public void reset() {
        latchedState = false;
    }
}