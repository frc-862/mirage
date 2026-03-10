package frc.util.overrunWatching;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.overrunWatching.RobotTimerInstance.Action;
import frc.util.overrunWatching.RobotTimerInstance.RobotTimer;

public abstract class TimedSubsystemBase extends SubsystemBase {
    
    @Override
    public final void periodic() {
        if (RobotTimer.timing()) {
            
            Time startTime = RobotController.getMeasureFPGATime();

            timedPeriodic();

            Time executionTime = RobotController.getMeasureFPGATime().minus(startTime);

            RobotTimer.get().record(new Action("Subsystems/" + getName() + "/Periodic"), executionTime);
        } else {
            timedPeriodic();
        }
    }

    public void timedPeriodic() {};
}
