package frc.util.leds;

import java.util.function.BooleanSupplier;

public class LEDLatch implements BooleanSupplier {
    private final BooleanSupplier condition;
    private boolean latchedState = false;

    public LEDLatch(BooleanSupplier condition) {
        this.condition = condition;
    }

    @Override
    public boolean getAsBoolean() {
        if (condition.getAsBoolean()) {
            latchedState = true;
        }
        return latchedState;
    }

    public void reset() {
        latchedState = false;
    }
}