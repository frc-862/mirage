package frc.util;

import java.util.EnumMap;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * A Command base class that implements a Finite State Machine.
 * Each state is defined as an independent method and registered
 * via {@link #configureStates()}.
 *
 * <p>Example usage:
 * <pre>
 * public class ClimbCommand extends StatefulCommand&lt;ClimbCommand.State&gt; {
 *     enum State { EXTENDING, RETRACTING, HOLDING }
 *
 *     public ClimbCommand(Climber climber) {
 *         super(State.EXTENDING);
 *         addRequirements(climber);
 *     }
 *
 *     &#64;Override
 *     protected void configureStates() {
 *         state(State.EXTENDING, this::extending)
 *             .onEnter(() -> climber.setDutyCycle(1d));
 *         state(State.RETRACTING, this::retracting)
 *             .onEnter(() -> climber.setDutyCycle(-1d));
 *         state(State.HOLDING, this::holding);
 *     }
 *
 *     private void extending() {
 *         if (climber.getForwardLimitSwitchState()) setState(State.RETRACTING);
 *     }
 *     private void retracting() {
 *         if (climber.getReverseLimitSwitchState()) setState(State.HOLDING);
 *     }
 *     private void holding() { setFinished(); }
 * }
 * </pre>
 *
 * @param <S> An enum type representing the states of the FSM
 */
public abstract class StatefulCommand<S extends Enum<S>> extends Command {

    private static class StateHandlers {
        Runnable onEnter;
        Runnable execute;
        Runnable onExit;

        StateHandlers(Runnable execute) {
            this.execute = execute;
        }
    }

    /**
     * Returned by {@link #state(Enum, Runnable)} to allow fluent
     * registration of onEnter/onExit callbacks.
     */
    protected class StateConfig {
        private final StateHandlers handlers;

        private StateConfig(StateHandlers handlers) {
            this.handlers = handlers;
        }

        /** Register a callback to run when entering this state. */
        public StateConfig onEnter(Runnable onEnter) {
            handlers.onEnter = onEnter;
            return this;
        }

        /** Register a callback to run when exiting this state. */
        public StateConfig onExit(Runnable onExit) {
            handlers.onExit = onExit;
            return this;
        }
    }

    private final S initialState;
    private final EnumMap<S, StateHandlers> handlerMap;
    private S currentState;
    private boolean finished;

    /**
     * Creates a new StatefulCommand.
     *
     * @param initialState The state the FSM starts in when the command is initialized
     */
    @SuppressWarnings("unchecked")
    protected StatefulCommand(S initialState) {
        this.initialState = initialState;
        this.handlerMap = new EnumMap<>((Class<S>) initialState.getClass());
        configureStates();
    }

    /**
     * Register all states and their handlers here. Called once from the constructor.
     * Use {@link #state(Enum, Runnable)} to register each state.
     */
    protected abstract void configureStates();

    /**
     * Register a state with its execute method.
     *
     * @param stateEnum The state to register
     * @param execute   The method to call each cycle while in this state
     * @return A {@link StateConfig} for optionally chaining onEnter/onExit callbacks
     */
    protected StateConfig state(S stateEnum, Runnable execute) {
        StateHandlers handlers = new StateHandlers(execute);
        handlerMap.put(stateEnum, handlers);
        return new StateConfig(handlers);
    }

    /**
     * Transition to a new state. Calls the current state's onExit (if any),
     * then the new state's onEnter (if any).
     *
     * @param newState The state to transition to
     */
    protected void setState(S newState) {
        if (currentState != null) {
            StateHandlers current = handlerMap.get(currentState);
            if (current != null && current.onExit != null) {
                current.onExit.run();
            }
        }

        currentState = newState;

        StateHandlers next = handlerMap.get(newState);
        if (next != null && next.onEnter != null) {
            next.onEnter.run();
        }
    }

    /** Returns the current state of the FSM. */
    protected S getState() {
        return currentState;
    }

    /** Call this from a state method to signal that the command should end. */
    protected void setFinished() {
        finished = true;
    }

    @Override
    public void initialize() {
        finished = false;
        currentState = null;
        setState(initialState);
    }

    @Override
    public void execute() {
        StateHandlers handlers = handlerMap.get(currentState);
        if (handlers != null && handlers.execute != null) {
            handlers.execute.run();
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        if (currentState != null) {
            StateHandlers handlers = handlerMap.get(currentState);
            if (handlers != null && handlers.onExit != null) {
                handlers.onExit.run();
            }
        }
    }
}
