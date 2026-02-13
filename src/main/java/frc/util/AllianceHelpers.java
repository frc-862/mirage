package frc.util;

import edu.wpi.first.wpilibj.DriverStation;

public class AllianceHelpers {

    /**
     * @return true if blue alliance or unknown, false if red alliance
    */
    public static boolean isBlueAlliance() {
        return (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue);
    }

    /**
     * @return true if red alliance, false if blue alliance or unknown
    */
    public static boolean isRedAlliance() {
        return !isBlueAlliance();
    }
}