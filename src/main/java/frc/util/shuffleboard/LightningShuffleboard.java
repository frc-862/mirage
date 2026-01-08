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

import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class LightningShuffleboard {
    private static HashMap<String, Object> keyList = new HashMap<String, Object>();

    //seperate hm for poses in order to retain publishers.
    private static HashMap<String, StructPublisher<Pose2d>> poseList = new HashMap<String, StructPublisher<Pose2d>>();

    /**
     * Creates and sets a double to NT through shuffleboard
     * @param tabName the tab this shuffleboard entry will be placed in
     * @param key the name of the shuffleboard entry
     * @param value the value of the shuffleboard entry
     * @implNote must be called periodically to update
     */
    public static void setDouble(String tabName, String key, double value) {
        ShuffleboardTab tab = Shuffleboard.getTab(tabName);

        String index = tabName + "/" + key;

        /* logic breakdown:
         * if the key does not exist, create it
         * if they exists but is not updated, update it
         * else, the key exists and is up-to-date, so nothing needs to be done
         */
        if(!keyList.containsKey(index)) {
            keyList.put(index, value);
            tab.add(key, value);
        } else if(!keyList.get(index).equals(value)) {
            keyList.put(index, value);
            NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable(tabName).getEntry(key).setDouble(value);
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
        ShuffleboardTab tab = Shuffleboard.getTab(tabName);

        String index = tabName + "/" + key;

        /* logic breakdown:
         * if the key does not exist, create it
         * if they exists but is not updated, update it
         * else, the key exists and is up-to-date, so nothing needs to be done
         */
        if(!keyList.containsKey(index)) {
            keyList.put(index, value);
            tab.add(key, value);
        } else if(!keyList.get(index).equals(value)) {
            keyList.put(index, value);
            NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable(tabName).getEntry(key).setBoolean(value);
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
        ShuffleboardTab tab = Shuffleboard.getTab(tabName);

        String index = tabName + "/" + key;

        /* logic breakdown:
         * if the key does not exist, create it
         * if they exists but is not updated, update it
         * else, the key exists and is up-to-date, so nothing needs to be done
         */
        if(!keyList.containsKey(index)) {
            keyList.put(index, value);
            tab.add(key, value);
        } else if(!keyList.get(index).equals(value)) {
            keyList.put(index, value);
            NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable(tabName).getEntry(key).setString(value);
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
        ShuffleboardTab tab = Shuffleboard.getTab(tabName);

        String index = tabName + "/" + key;

        /* logic breakdown:
         * if the key does not exist, create it. this is it because its a supplier
         */
        if(!keyList.containsKey(index)) {
            keyList.put(index, value);
            tab.add(key, value);
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
        ShuffleboardTab tab = Shuffleboard.getTab(tabName);

        String index = tabName + "/" + key;

        /* logic breakdown:
         * if the key does not exist, create it. this is it because its a supplier
         */
        if(!keyList.containsKey(index)) {
            keyList.put(index, value);
            tab.add(key, value);
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
        ShuffleboardTab tab = Shuffleboard.getTab(tabName);

        String index = tabName + "/" + key;

        /* logic breakdown:
         * if the key does not exist, create it. this is it because its a supplier
         */
        if(!keyList.containsKey(index)) {
            keyList.put(index, value);
            tab.add(key, value);
        }
    }

    /**
     * Creates and grabs a double from NT through shuffleboard 
     * @param tabName the tab to grab the value from
     * @param key the name of the shuffleboard entry
     * @param defaultValue the initial entry value
     * @return the value of the shuffleboard entry
     * 
     * @implNote this causes some performance issues if used periodically.
     */
    public static double getDouble(String tabName, String key, double defaultValue) {
        String index = tabName + "/" + key;

        /* logic breakdown:
         * if the key does not exist, create it
         * if the key exists, update it
         * this does create some overhead if used periodically
         */
        if(!keyList.containsKey(index)) {
            keyList.put(index, defaultValue);
            Shuffleboard.getTab(tabName).add(key, defaultValue);
            return defaultValue;
        } else {
            double value = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable(tabName).getEntry(key).getDouble(defaultValue);
            keyList.put(index, value);
            return value;
        }
    }

    /**
     * Creates and grabs a boolean from NT through shuffleboard
     * @param tabName the tab to grab the value from
     * @param key the name of the shuffleboard entry
     * @param defaultValue the initial entry value
     * @return the value of the shuffleboard entry
     * 
     * @implNote this causes some performance issues if used periodically.
     */
    public static boolean getBool(String tabName, String key, boolean defaultValue) {
        String index = tabName + "/" + key;

        /* logic breakdown:
         * if the key does not exist, create it
         * if the key exists, update it
         * this does create some overhead if used periodically
         */
        if(!keyList.containsKey(index)) {
            keyList.put(index, defaultValue);
            Shuffleboard.getTab(tabName).add(key, defaultValue);
            return defaultValue;
        } else {
            boolean value = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable(tabName).getEntry(key).getBoolean(defaultValue);
            keyList.put(index, value);
            return value;
        }
    }

    /**
     * Creates and grabs a string from NT through shuffleboard
     * @param tabName the tab to grab the value from
     * @param key the name of the shuffleboard entry
     * @param defaultValue the initial entry value
     * @return the value of the shuffleboard entry
     * 
     * @implNote this causes some performance issues if used periodically.
     */
    public static String getString(String tabName, String key, String defaultValue) {
        String index = tabName + "/" + key;

        /* logic breakdown:
         * if the key does not exist, create it
         * if the key exists, update it
         * this does create some overhead if used periodically
         */
        if(!keyList.containsKey(index)) {
            keyList.put(index, defaultValue);
            Shuffleboard.getTab(tabName).add(key, defaultValue);
            return defaultValue;
        } else {
            String value = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable(tabName).getEntry(key).getString(defaultValue);
            keyList.put(index, value);
            return value;
        }
    }

    
    /**
     * Creates and sets a double array from NT through shuffleboard
     * @param tabName the tab to set the value to
     * @param key the name of the shuffleboard entry
     * @param value the value of the shuffleboard entry
     * @implNote must be called periodically to update
     */
    public static void setDoubleArray(String tabName, String key, double[] value) {
        ShuffleboardTab tab = Shuffleboard.getTab(tabName);

        String index = tabName + "/" + key;

        /* logic breakdown:
         * if the key does not exist, create it
         * if they exists but is not updated, update it
         * else, the key exists and is up-to-date, so nothing needs to be done
         */
        if(!keyList.containsKey(index)) {
            keyList.put(index, value);
            tab.add(key, value);
        } else if(!keyList.get(index).equals(value)) {
            keyList.put(index, value);
            NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable(tabName).getEntry(key).setDoubleArray(value);
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
        ShuffleboardTab tab = Shuffleboard.getTab(tabName);

        String index = tabName + "/" + key;

        /* logic breakdown:
         * if the key does not exist, create it
         * if they exists but is not updated, update it
         * else, the key exists and is up-to-date, so nothing needs to be done
         */
        if(!keyList.containsKey(index)) {
            keyList.put(index, value);
            tab.add(key, value);
        } else if(!keyList.get(index).equals(value)) {
            keyList.put(index, value);
            NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable(tabName).getEntry(key).setBooleanArray(value);
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
        ShuffleboardTab tab = Shuffleboard.getTab(tabName);

        String index = tabName + "/" + key;

        /* logic breakdown:
         * if the key does not exist, create it
         * if they exists but is not updated, update it
         * else, the key exists and is up-to-date, so nothing needs to be done
         */
        if(!keyList.containsKey(index)) {
            keyList.put(index, value);
            tab.add(key, value);
        } else if(!keyList.get(index).equals(value)) {
            keyList.put(index, value);
            NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable(tabName).getEntry(key).setStringArray(value);
        }
    }

    /**
     * Creates and sets a Pose2d from NT through shuffleboard in AdvantageScope Struct formar
     * @param tabName the tab to set the value to
     * @param key the name of the shuffleboard entry
     * @param value the value of the shuffleboard entry
     * @implNote must be called periodically to update
     */
    public static void setPose2d(String tabName, String key, Pose2d value) {
        // ShuffleboardTab tab = Shuffleboard.getTab(tabName);

        String index = tabName + "/" + key;

        /* logic breakdown:
         * if the key does not exist, create it
         * if they exists but is not updated, update it
         * else, the key exists and is up-to-date, so nothing needs to be done
         */
        if(!keyList.containsKey(index)) {
            keyList.put(index, value);
            poseList.put(index, NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable(tabName).getStructTopic(key, Pose2d.struct).publish());
        } else if(!keyList.get(index).equals(value)) {
            keyList.put(index, value);
            poseList.get(index).accept(value);
        }
    }

    /**
     * Set a {@link <a href="https://docs.wpilib.org/en/stable/docs/software/telemetry/robot-telemetry-with-sendable.html">Sendable</a>} object to NT through shuffleboard
     * @param tabName the tab this shuffleboard entry will be placed in
     * @param key the name of the shuffleboard entry
     * @param value the value of the shuffleboard entry
     */
    public static void send(String tabName, String key, Sendable value) {
        ShuffleboardTab tab = Shuffleboard.getTab(tabName);

        String index = tabName + "/" + key;

        /* logic breakdown:
         * if the key does not exist, create it. this is it because its a supplier
         */
        if(!keyList.containsKey(index)) {
            keyList.put(index, value);
            tab.add(key, value);
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
        // if the component exists, get it using the NetworkTable, and if not, create the component
        try {
            return NetworkTableInstance.getDefault().getTable(table).getSubTable(subTable).getEntry(key);
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
        ShuffleboardTab tab = Shuffleboard.getTab(tabName);

        String index = tabName + "/" + key;

        /* logic breakdown:
         * if the key does not exist, create it
         * if they exists but is not updated, update it
         * else, the key exists and is up-to-date, so nothing needs to be done
         */
        if(!keyList.containsKey(index)) {
            keyList.put(index, value);
            tab.add(key, value);
        } else if(!keyList.get(index).equals(value)) {
            keyList.put(index, value);
            NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable(tabName).getEntry(key).setValue(value);
        }
    }

    /**
     * @deprecated use {@link #send(String, String, Sendable)} instead
     * 
     * Set a {@link <a href="https://docs.wpilib.org/en/stable/docs/software/telemetry/robot-telemetry-with-sendable.html">Sendable</a>} object to NT through shuffleboard
     * @param tabName the tab this shuffleboard entry will be placed in
     * @param key the name of the shuffleboard entry
     * @param value the value of the shuffleboard entry
     */
    public static void set(String tabName, String key, Sendable value) {
        ShuffleboardTab tab = Shuffleboard.getTab(tabName);

        String index = tabName + "/" + key;

        /* logic breakdown:
         * if the key does not exist, create it. this is it because its a supplier
         */
        if(!keyList.containsKey(index)) {
            keyList.put(index, value);
            tab.add(key, value);
        }
    }
}
