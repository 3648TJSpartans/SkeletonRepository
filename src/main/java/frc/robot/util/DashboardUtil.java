// Copyright (c) 2025-2026 First Robotics Competition Team 3648.
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.util;

import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;

/** Lightweight SmartDashboard replacement backed by NetworkTables for WPILib 2027. */
public final class DashboardUtil {
  private static final NetworkTable table =
      NetworkTableInstance.getDefault().getTable("SmartDashboard");

  private DashboardUtil() {}

  public static void putNumber(String key, double value) {
    table.getEntry(key).setDouble(value);
  }

  public static double getNumber(String key, double defaultValue) {
    return table.getEntry(key).getDouble(defaultValue);
  }

  public static void putBoolean(String key, boolean value) {
    table.getEntry(key).setBoolean(value);
  }

  public static boolean getBoolean(String key, boolean defaultValue) {
    return table.getEntry(key).getBoolean(defaultValue);
  }
}
