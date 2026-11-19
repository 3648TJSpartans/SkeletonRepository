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

package frc.robot.util.miscTunables;

import static frc.robot.util.TuningUpdater.*;

import frc.robot.util.DashboardUtil;
import frc.robot.util.Tunable;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Rotation3d;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class TunableRotation3d extends Tunable<Rotation3d> {

  /**
   * Create a new TunableRotation3d
   *
   * @param dashboardKey Key on dashboard
   */
  public TunableRotation3d(String dashboardKey) {
    super(dashboardKey);
  }

  /**
   * Create a new TunableRotation3d with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public TunableRotation3d(String dashboardKey, Rotation3d defaultValue) {
    super(dashboardKey, defaultValue);
  }

  public TunableRotation3d(String dashboardKey, Rotation3d defaultValue, Runnable update) {
    super(dashboardKey, defaultValue, update);
  }

  @Override
  protected void putDashboardValue(String key, Rotation3d defaultValue) {
    DashboardUtil.putNumber(
        key + "/pitch", DashboardUtil.getNumber(key + "/pitch", defaultValue.getX()));
    DashboardUtil.putNumber(
        key + "/yaw", DashboardUtil.getNumber(key + "/yaw", defaultValue.getY()));
    DashboardUtil.putNumber(
        key + "/roll", DashboardUtil.getNumber(key + "/roll", defaultValue.getZ()));
  }

  @Override
  protected Rotation3d getDashboardValue(String key, Rotation3d defaultValue) {
    return TUNING_MODE
        ? new Rotation3d(
            DashboardUtil.getNumber(key + "/pitch", defaultValue.getX()),
            DashboardUtil.getNumber(key + "/yaw", defaultValue.getY()),
            DashboardUtil.getNumber(key + "/roll", defaultValue.getZ()))
        : defaultValue;
  }

  @Override
  protected void logValue(
      String key, boolean TUNING_MODE, Rotation3d value, Rotation3d defaultValue) {
    // System.out.println("Log Value run");
    Logger.recordOutput(key, getDashboardValue(key, defaultValue));
  }
}
