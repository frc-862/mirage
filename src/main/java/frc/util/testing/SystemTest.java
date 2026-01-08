package frc.util.testing;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SystemTest {

    private static ShuffleboardTab systemTestTab = Shuffleboard.getTab("SystemTest");;

    private static GenericEntry interrupt = systemTestTab.add("interrupt", false)
            .withWidget(BuiltInWidgets.kToggleButton).getEntry();

    private static final HashMap<String, Command> systemTests = new HashMap<>();

    /**
     * Registers a system test to be sent to the dashboard
     * 
     * @param name    title for the dashboard widget
     * @param command command to be run from the dashboard
     */
    public static void registerTest(String name, SystemTestCommand systemTestCmd) {
        systemTests.put(name, systemTestCmd.until(() -> interrupt.getBoolean(false))
            .beforeStarting(new InstantCommand(() -> SystemTest.cancelAllTests(name))));
    }

    /**
     * Loads system tests to system test tab on shuffleboard
     */
    public static void loadTests() {
        for (Map.Entry<String, Command> entry : systemTests.entrySet()) {
            systemTestTab.add(entry.getKey(), entry.getValue());
        }
    }

    /**
     * Cancels all system tests
     */
    public static void cancelAllTests(String exceptionName) {
        for (Entry<String, Command> entry  : systemTests.entrySet()) {
            if (!entry.getKey().equals(exceptionName)) {
                entry.getValue().end(true);
                entry.getValue().cancel();
            }
        }
    }
    public static void cancelAllTests() {
        cancelAllTests("");
    }

}