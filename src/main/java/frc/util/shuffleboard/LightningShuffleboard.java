/*
* Lightning Shuffleboard - a simple, easy-to-use shuffleboard library for FRC teams
* Copyright (C) 2024 Lightning Robotics
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

package frc.util.shuffleboard;

import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class LightningShuffleboard {
    // Two-level caches keyed by [tabName][key] — avoids string concatenation on every call
    private static final HashMap<String, HashMap<String, Object>> valueCache = new HashMap<>();
    private static final HashMap<String, HashMap<String, NetworkTableEntry>> entryCache = new HashMap<>();
    private static final HashMap<String, HashSet<String>> registeredKeys = new HashMap<>();

    // Cached struct publishers
    private static final HashMap<String, HashMap<String, StructPublisher<Pose2d>>> posePublishers = new HashMap<>();
    private static final HashMap<String, HashMap<String, StructPublisher<Pose3d>>> pose3dPublishers = new HashMap<>();
    private static final HashMap<String, HashMap<String, StructPublisher<Translation2d>>> translation2dPublishers = new HashMap<>();

    private static <T> T getFromCache(HashMap<String, HashMap<String, T>> cache, String tab, String key) {
        HashMap<String, T> tabMap = cache.get(tab);
        return tabMap == null ? null : tabMap.get(key);
    }

    private static <T> void putInCache(HashMap<String, HashMap<String, T>> cache, String tab, String key, T value) {
        cache.computeIfAbsent(tab, k -> new HashMap<>()).put(key, value);
    }

    private static NetworkTableEntry getOrCreateEntry(String tabName, String key, Object initialValue) {
        NetworkTableEntry entry = getFromCache(entryCache, tabName, key);
        if (entry == null) {
            Shuffleboard.getTab(tabName).add(key, initialValue);
            entry = NetworkTableInstance.getDefault()
                .getTable("Shuffleboard").getSubTable(tabName).getEntry(key);
            putInCache(entryCache, tabName, key, entry);
        }
        return entry;
    }

    private static boolean isFirstRegistration(String tabName, String key) {
        return registeredKeys.computeIfAbsent(tabName, k -> new HashSet<>()).add(key);
    }

    /**
     * Creates and sets a double to NT through shuffleboard
     * @param tabName the tab this shuffleboard entry will be placed in
     * @param key the name of the shuffleboard entry
     * @param value the value of the shuffleboard entry
     * @implNote must be called periodically to update
     */
    public static void setDouble(String tabName, String key, double value) {
        NetworkTableEntry entry = getFromCache(entryCache, tabName, key);
        if (entry == null) {
            entry = getOrCreateEntry(tabName, key, value);
            putInCache(valueCache, tabName, key, value);
        } else {
            double last = (double) getFromCache(valueCache, tabName, key);
            if (last != value) {
                entry.setDouble(value);
                putInCache(valueCache, tabName, key, value);
            }
        }
    }

    /**
     * Creates and sets a boolean to NT through shuffleboard
     * @param tabName the tab this shuffleboard entry will be placed in
     * @param key the name of the shuffleboard entry
     * @param value the value of the shuffleboard entry
     * @implNote must be called periodically to update
     */
    public static void setBool(String tabName, String key, boolean value) {
        NetworkTableEntry entry = getFromCache(entryCache, tabName, key);
        if (entry == null) {
            entry = getOrCreateEntry(tabName, key, value);
            putInCache(valueCache, tabName, key, value);
        } else {
            boolean last = (boolean) getFromCache(valueCache, tabName, key);
            if (last != value) {
                entry.setBoolean(value);
                putInCache(valueCache, tabName, key, value);
            }
        }
    }

    /**
     * Creates and sets a string to NT through shuffleboard
     * @param tabName the tab this shuffleboard entry will be placed in
     * @param key the name of the shuffleboard entry
     * @param value the value of the shuffleboard entry
     * @implNote must be called periodically to update
     */
    public static void setString(String tabName, String key, String value) {
        NetworkTableEntry entry = getFromCache(entryCache, tabName, key);
        if (entry == null) {
            entry = getOrCreateEntry(tabName, key, value);
            putInCache(valueCache, tabName, key, value);
        } else {
            Object last = getFromCache(valueCache, tabName, key);
            if (!value.equals(last)) {
                entry.setString(value);
                putInCache(valueCache, tabName, key, value);
            }
        }
    }

    /**
     * Creates and sets a double supplier to NT through shuffleboard
     * @param tabName the tab this shuffleboard entry will be placed in
     * @param key the name of the shuffleboard entry
     * @param value the value of the shuffleboard entry
     * @implNote will update automatically
     */
    public static void setDoubleSupplier(String tabName, String key, DoubleSupplier value) {
        if (isFirstRegistration(tabName, key)) {
            Shuffleboard.getTab(tabName).add(key, value);
        }
    }

    /**
     * Creates and sets a boolean supplier to NT through shuffleboard
     * @param tabName the tab this shuffleboard entry will be placed in
     * @param key the name of the shuffleboard entry
     * @param value the value of the shuffleboard entry
     * @implNote will update automatically
     */
    public static void setBoolSupplier(String tabName, String key, BooleanSupplier value) {
        if (isFirstRegistration(tabName, key)) {
            Shuffleboard.getTab(tabName).add(key, value);
        }
    }

    /**
     * Creates and sets a string supplier to NT through shuffleboard
     * @param tabName the tab this shuffleboard entry will be placed in
     * @param key the name of the shuffleboard entry
     * @param value the value of the shuffleboard entry
     * @implNote will update automatically
     */
    public static void setStringSupplier(String tabName, String key, Supplier<String> value) {
        if (isFirstRegistration(tabName, key)) {
            Shuffleboard.getTab(tabName).add(key, value);
        }
    }

    /**
     * Creates and grabs a double from NT through shuffleboard
     * @param tabName the tab to grab the value from
     * @param key the name of the shuffleboard entry
     * @param defaultValue the initial entry value
     * @return the value of the shuffleboard entry
     */
    public static double getDouble(String tabName, String key, double defaultValue) {
        NetworkTableEntry entry = getFromCache(entryCache, tabName, key);
        if (entry == null) {
            entry = getOrCreateEntry(tabName, key, defaultValue);
            return defaultValue;
        }
        return entry.getDouble(defaultValue);
    }

    /**
     * Creates and grabs a boolean from NT through shuffleboard
     * @param tabName the tab to grab the value from
     * @param key the name of the shuffleboard entry
     * @param defaultValue the initial entry value
     * @return the value of the shuffleboard entry
     */
    public static boolean getBool(String tabName, String key, boolean defaultValue) {
        NetworkTableEntry entry = getFromCache(entryCache, tabName, key);
        if (entry == null) {
            entry = getOrCreateEntry(tabName, key, defaultValue);
            return defaultValue;
        }
        return entry.getBoolean(defaultValue);
    }

    /**
     * Creates and grabs a string from NT through shuffleboard
     * @param tabName the tab to grab the value from
     * @param key the name of the shuffleboard entry
     * @param defaultValue the initial entry value
     * @return the value of the shuffleboard entry
     */
    public static String getString(String tabName, String key, String defaultValue) {
        NetworkTableEntry entry = getFromCache(entryCache, tabName, key);
        if (entry == null) {
            entry = getOrCreateEntry(tabName, key, defaultValue);
            return defaultValue;
        }
        return entry.getString(defaultValue);
    }

    /**
     * Creates and sets a double array from NT through shuffleboard
     * @param tabName the tab to set the value to
     * @param key the name of the shuffleboard entry
     * @param value the value of the shuffleboard entry
     * @implNote must be called periodically to update
     */
    public static void setDoubleArray(String tabName, String key, double[] value) {
        NetworkTableEntry entry = getFromCache(entryCache, tabName, key);
        if (entry == null) {
            entry = getOrCreateEntry(tabName, key, value);
            putInCache(valueCache, tabName, key, value);
        } else if (!Arrays.equals((double[]) getFromCache(valueCache, tabName, key), value)) {
            entry.setDoubleArray(value);
            putInCache(valueCache, tabName, key, value);
        }
    }

    /**
     * Creates and sets a boolean array from NT through shuffleboard
     * @param tabName the tab to set the value to
     * @param key the name of the shuffleboard entry
     * @param value the value of the shuffleboard entry
     * @implNote must be called periodically to update
     */
    public static void setBoolArray(String tabName, String key, boolean[] value) {
        NetworkTableEntry entry = getFromCache(entryCache, tabName, key);
        if (entry == null) {
            entry = getOrCreateEntry(tabName, key, value);
            putInCache(valueCache, tabName, key, value);
        } else if (!Arrays.equals((boolean[]) getFromCache(valueCache, tabName, key), value)) {
            entry.setBooleanArray(value);
            putInCache(valueCache, tabName, key, value);
        }
    }

    /**
     * Creates and sets a string array from NT through shuffleboard
     * @param tabName the tab to set the value to
     * @param key the name of the shuffleboard entry
     * @param value the value of the shuffleboard entry
     * @implNote must be called periodically to update
     */
    public static void setStringArray(String tabName, String key, String[] value) {
        NetworkTableEntry entry = getFromCache(entryCache, tabName, key);
        if (entry == null) {
            entry = getOrCreateEntry(tabName, key, value);
            putInCache(valueCache, tabName, key, value);
        } else if (!Arrays.equals((String[]) getFromCache(valueCache, tabName, key), value)) {
            entry.setStringArray(value);
            putInCache(valueCache, tabName, key, value);
        }
    }

    /**
     * Creates and sets a Pose2d from NT through shuffleboard in AdvantageScope Struct format
     * @param tabName the tab to set the value to
     * @param key the name of the shuffleboard entry
     * @param value the value of the shuffleboard entry
     * @implNote must be called periodically to update
     */
    public static void setPose2d(String tabName, String key, Pose2d value) {
        StructPublisher<Pose2d> publisher = getFromCache(posePublishers, tabName, key);
        if (publisher == null) {
            publisher = NetworkTableInstance.getDefault()
                .getTable("Shuffleboard").getSubTable(tabName)
                .getStructTopic(key, Pose2d.struct).publish();
            putInCache(posePublishers, tabName, key, publisher);
            putInCache(valueCache, tabName, key, value);
            publisher.accept(value);
        } else if (!value.equals(getFromCache(valueCache, tabName, key))) {
            putInCache(valueCache, tabName, key, value);
            publisher.accept(value);
        }
    }

    /**
     * Creates and sets a Translation2d from NT through shuffleboard in AdvantageScope Struct format
     * @param tabName the tab to set the value to
     * @param key the name of the shuffleboard entry
     * @param value the value of the shuffleboard entry
     * @implNote must be called periodically to update
     */
    public static void setTranslation2d(String tabName, String key, Translation2d value) {
        StructPublisher<Translation2d> publisher = getFromCache(translation2dPublishers, tabName, key);
        if (publisher == null) {
            publisher = NetworkTableInstance.getDefault()
                .getTable("Shuffleboard").getSubTable(tabName)
                .getStructTopic(key, Translation2d.struct).publish();
            putInCache(translation2dPublishers, tabName, key, publisher);
            putInCache(valueCache, tabName, key, value);
            publisher.accept(value);
        } else if (!value.equals(getFromCache(valueCache, tabName, key))) {
            putInCache(valueCache, tabName, key, value);
            publisher.accept(value);
        }
    }

    /**
     * Creates and sets a Pose3d from NT through shuffleboard in AdvantageScope Struct format
     * @param tabName the tab to set the value to
     * @param key the name of the shuffleboard entry
     * @param value the value of the shuffleboard entry
     * @implNote must be called periodically to update
     */
    public static void setPose3d(String tabName, String key, Pose3d value) {
        StructPublisher<Pose3d> publisher = getFromCache(pose3dPublishers, tabName, key);
        if (publisher == null) {
            publisher = NetworkTableInstance.getDefault()
                .getTable("Shuffleboard").getSubTable(tabName)
                .getStructTopic(key, Pose3d.struct).publish();
            putInCache(pose3dPublishers, tabName, key, publisher);
            putInCache(valueCache, tabName, key, value);
            publisher.accept(value);
        } else if (!value.equals(getFromCache(valueCache, tabName, key))) {
            putInCache(valueCache, tabName, key, value);
            publisher.accept(value);
        }
    }

    /**
     * Set a {@link <a href="https://docs.wpilib.org/en/stable/docs/software/telemetry/robot-telemetry-with-sendable.html">Sendable</a>} object to NT through shuffleboard
     * @param tabName the tab this shuffleboard entry will be placed in
     * @param key the name of the shuffleboard entry
     * @param value the value of the shuffleboard entry
     */
    public static void send(String tabName, String key, Sendable value) {
        if (isFirstRegistration(tabName, key)) {
            Shuffleboard.getTab(tabName).add(key, value);
        }
    }

    /**
     * grabs a NT entry
     * @param table the table to grab the entry from ("Shuffleboard" for shuffleboard)
     * @param subTable the subtable to grab the entry from (tab name for shuffleboard)
     * @param key the name of the NT entry
     * @return the NT entry
     *
     * @implNote if entry does not exist, returns null. will NOT create entry automatically.
     */
    public static NetworkTableEntry getEntry(String table, String subTable, String key) {
        // getEntry uses a different key space (arbitrary table), so uses flat lookup
        NetworkTableEntry entry = getFromCache(entryCache, table + "/" + subTable, key);
        if (entry != null) {
            return entry;
        }
        try {
            entry = NetworkTableInstance.getDefault().getTable(table).getSubTable(subTable).getEntry(key);
            putInCache(entryCache, table + "/" + subTable, key, entry);
            return entry;
        } catch (Exception e) {
            System.out.println("entry grab failed: " + e);
            return null;
        }
    }

    /**
     * Set a generic object to NT through shuffleboard
     * @param tabName the tab this shuffleboard entry will be placed in
     * @param key the name of the shuffleboard entry
     * @param value the value of the shuffleboard entry
     */
    public static void set(String tabName, String key, Object value) {
        NetworkTableEntry entry = getFromCache(entryCache, tabName, key);
        if (entry == null) {
            entry = getOrCreateEntry(tabName, key, value);
            putInCache(valueCache, tabName, key, value);
        } else if (!value.equals(getFromCache(valueCache, tabName, key))) {
            entry.setValue(value);
            putInCache(valueCache, tabName, key, value);
        }
    }

    /**
     * Set a {@link <a href="https://docs.wpilib.org/en/stable/docs/software/telemetry/robot-telemetry-with-sendable.html">Sendable</a>} object to NT through shuffleboard
     * @param tabName the tab this shuffleboard entry will be placed in
     * @param key the name of the shuffleboard entry
     * @param value the value of the shuffleboard entry
     */
    public static void set(String tabName, String key, Sendable value) {
        if (isFirstRegistration(tabName, key)) {
            Shuffleboard.getTab(tabName).add(key, value);
        }
    }
}
