package frc.util;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestSubsytem extends SubsystemBase {
    double counter = 0;
    @Override
    public void periodic() {
        counter ++;
        double startTime = System.currentTimeMillis();

        // run stuff

        if (counter % 5 == 0) {
            DataLogManager.log("[TestSubsytem]" + " Periodic Time: " + (System.currentTimeMillis() - startTime) + "ms" + " Counter: " + counter);
        }
    }
}