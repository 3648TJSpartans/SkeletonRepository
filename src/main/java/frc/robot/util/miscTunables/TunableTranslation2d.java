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
import org.wpilib.math.geometry.Translation2d;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class TunableTranslation2d extends Tunable<Translation2d> {

  /**
   * Create a new TunableTranslation2d
   *
   * @param dashboardKey Key on dashboard
   */
  public TunableTranslation2d(String dashboardKey) {
    super(dashboardKey);
  }

  /**
   * Create a new TunableTranslation2d with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public TunableTranslation2d(String dashboardKey, Translation2d defaultValue) {
    super(dashboardKey, defaultValue);
  }

  public TunableTranslation2d(String dashboardKey, Translation2d defaultValue, Runnable update) {
    super(dashboardKey, defaultValue, update);
  }

  @Override
  protected void putDashboardValue(String key, Translation2d defaultValue) {
    DashboardUtil.putNumber(key + "/x", DashboardUtil.getNumber(key + "/x", defaultValue.getX()));
    DashboardUtil.putNumber(key + "/y", DashboardUtil.getNumber(key + "/y", defaultValue.getY()));
  }

  @Override
  protected Translation2d getDashboardValue(String key, Translation2d defaultValue) {
    return TUNING_MODE
        ? new Translation2d(
            DashboardUtil.getNumber(key + "/x", defaultValue.getX()),
            DashboardUtil.getNumber(key + "/y", defaultValue.getY()))
        : defaultValue;
  }

  @Override
  protected void logValue(
      String key, boolean TUNING_MODE, Translation2d value, Translation2d defaultValue) {
    Logger.recordOutput(key, getDashboardValue(key, defaultValue));
  }
}
