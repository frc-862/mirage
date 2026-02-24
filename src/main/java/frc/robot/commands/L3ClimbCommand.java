package frc.robot.commands;

import frc.robot.subsystems.Climber;
import frc.util.StatefulCommand;

/**
 * Extends and retracts the climber 3 times to achieve a level 3 climb.
 * Stops the motor when the command ends (either finished or interrupted).
 */
public class L3ClimbCommand extends StatefulCommand<L3ClimbCommand.State> {

    enum State { EXTENDING, RETRACTING }

    private static final int TOTAL_CYCLES = 3;

    private final Climber climber;
    private int cyclesCompleted;

    public L3ClimbCommand(Climber climber) {
        super(State.EXTENDING);
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    protected void configureStates() {
        state(State.EXTENDING, this::extending)
            .onEnter(() -> climber.setDutyCycle(1d));

        state(State.RETRACTING, this::retracting)
            .onEnter(() -> climber.setDutyCycle(-1d));
    }

    @Override
    public void initialize() {
        cyclesCompleted = 0;
        super.initialize();
    }

    private void extending() {
        if (climber.getForwardLimitSwitchState()) {
            setState(State.RETRACTING);
        }
    }

    private void retracting() {
        if (climber.getReverseLimitSwitchState()) {
            cyclesCompleted++;
            if (cyclesCompleted >= TOTAL_CYCLES) {
                setFinished();
            } else {
                setState(State.EXTENDING);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        climber.stop();
    }
}
