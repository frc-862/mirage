package frc.util.overrunWatching;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.util.overrunWatching.RobotTimerInstance.Action;
import frc.util.overrunWatching.RobotTimerInstance.RobotTimer;

public class TimedCommand extends WrapperCommand {

    private TimedCommand(Command command) {
        super(command);
    }

    @Override
    public void initialize() {
        Time startTime = RobotController.getMeasureFPGATime();
        super.initialize();
        Time executionTime = RobotController.getMeasureFPGATime().minus(startTime);
        RobotTimer.get().record(new Action("Commands/" + getName() + "/Initialize"), executionTime);
    }

    @Override
    public void execute() {
        Time startTime = RobotController.getMeasureFPGATime();
        super.execute();
        Time executionTime = RobotController.getMeasureFPGATime().minus(startTime);
        RobotTimer.get().record(new Action("Commands/" + getName() + "/Execute"), executionTime);
    }

    @Override
    public void end(boolean interrupted) {
        Time startTime = RobotController.getMeasureFPGATime();
        super.end(interrupted);
        Time executionTime = RobotController.getMeasureFPGATime().minus(startTime);
        RobotTimer.get().record(new Action("Commands/" + getName() + "/End"), executionTime);
    }

    public static Command time(Command command) {
        if (RobotTimer.timing()) {
            return new TimedCommand(command);
        } else {
            return command;
        }
    }
    
}
