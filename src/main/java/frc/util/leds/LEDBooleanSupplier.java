package frc.util.leds;

import java.util.function.BooleanSupplier;

public class LEDBooleanSupplier implements BooleanSupplier {

    private boolean called = false;
    private final BooleanSupplier t;

    public LEDBooleanSupplier(BooleanSupplier t) {
        this.t = t;
    }

    @Override
    public boolean getAsBoolean() {
        if (!called) {
            called = true;
            return !t.getAsBoolean();
        } else {
            return t.getAsBoolean();
        }
    }

}