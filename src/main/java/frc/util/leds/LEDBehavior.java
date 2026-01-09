package frc.util.leds;

import java.util.ArrayList;
import java.util.function.Consumer;

public class LEDBehavior {
    private final ArrayList<Consumer<LEDController>> behaviors;

    public LEDBehavior() {
        behaviors = new java.util.ArrayList<>();
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

    public void apply(LEDController leds) {
        behaviors.forEach(behavior -> behavior.accept(leds));
    }

}
