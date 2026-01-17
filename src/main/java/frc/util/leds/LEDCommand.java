package frc.util.leds;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public abstract class LEDCommand extends Command {
    private final ArrayList<Command> successCommands;
    private final ArrayList<Command> failCommands;

    public LEDCommand() {
        super();

        successCommands = new ArrayList<>();
        failCommands = new ArrayList<>();
    }

    public Command onSuccess(Command command) {
        successCommands.add(command);

        return this;
    }

    public Command onFail(Command command) {
        failCommands.add(command);

        return this;
    }

    public void succeeded(boolean success) {
         if (success) {
            for (Command command : successCommands) {
                CommandScheduler.getInstance().schedule(command);
            }
        } else {
            for (Command command : failCommands) {
                CommandScheduler.getInstance().schedule(command);
            }
        }
    }
}
