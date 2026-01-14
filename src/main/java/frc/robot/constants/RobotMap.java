package frc.robot.constants;

import java.nio.file.Paths;

public class RobotMap {
    
    public static final String OASIS_IDENTIFIER = "/home/lvuser/Oasis"; // Differentiate between Oasis and Mirage
    public static final boolean IS_OASIS = Paths.get(OASIS_IDENTIFIER).toFile().exists();

    public static final double UPDATE_FREQ = 0.02;
}
