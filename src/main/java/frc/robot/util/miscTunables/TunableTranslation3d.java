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

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Tunable;
import org.littletonrobotics.junction.Logger;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class TunableTranslation3d extends Tunable<Translation3d> {

  /**
   * Create a new TunableTranslation3d
   *
   * @param dashboardKey Key on dashboard
   */
  public TunableTranslation3d(String dashboardKey) {
    super(dashboardKey);
  }

  /**
   * Create a new TunableTranslation3d with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public TunableTranslation3d(String dashboardKey, Translation3d defaultValue) {
    super(dashboardKey, defaultValue);
  }

  public TunableTranslation3d(String dashboardKey, Translation3d defaultValue, Runnable update) {
    super(dashboardKey, defaultValue, update);
  }

  @Override
  protected void putDashboardValue(String key, Translation3d defaultValue) {
    SmartDashboard.putNumber(key + "/x", SmartDashboard.getNumber(key + "/x", defaultValue.getX()));
    SmartDashboard.putNumber(key + "/y", SmartDashboard.getNumber(key + "/y", defaultValue.getY()));
    SmartDashboard.putNumber(key + "/z", SmartDashboard.getNumber(key + "/z", defaultValue.getZ()));
  }

  @Override
  protected Translation3d getDashboardValue(String key, Translation3d defaultValue) {
    return TUNING_MODE
        ? new Translation3d(
            SmartDashboard.getNumber(key + "/x", defaultValue.getX()),
            SmartDashboard.getNumber(key + "/y", defaultValue.getY()),
            SmartDashboard.getNumber(key + "/z", defaultValue.getZ()))
        : defaultValue;
  }

  @Override
  protected void logValue(
      String key, boolean TUNING_MODE, Translation3d value, Translation3d defaultValue) {
    Logger.recordOutput(key, getDashboardValue(key, defaultValue));
  }
}
