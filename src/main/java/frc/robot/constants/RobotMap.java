package frc.robot.constants;

import java.nio.file.Paths;

public class RobotMap {
    
    public static final String OASIS_IDENTIFIER = "/home/lvuser/Oasis"; // Differentiate between Triton and Nautlius
    public static final boolean IS_OASIS = Paths.get(OASIS_IDENTIFIER).toFile().exists();
}
