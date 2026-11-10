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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Tunable;
import java.util.function.IntSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class TunableInteger extends Tunable<Integer> implements IntSupplier {

  /**
   * Create a new TunableInteger
   *
   * @param dashboardKey Key on dashboard
   */
  public TunableInteger(String dashboardKey) {
    super(dashboardKey);
  }

  /**
   * Create a new TunableInteger with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public TunableInteger(String dashboardKey, int defaultValue) {
    super(dashboardKey, defaultValue);
  }

  public TunableInteger(String dashboardKey, int defaultValue, Runnable update) {
    super(dashboardKey, defaultValue, update);
  }

  @Override
  protected void putDashboardValue(String key, Integer defaultValue) {
    SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, defaultValue));
  }

  @Override
  protected Integer getDashboardValue(String key, Integer defaultValue) {
    return TUNING_MODE ? (int) SmartDashboard.getNumber(key, defaultValue) : defaultValue;
  }

  @Override
  protected void logValue(String key, boolean TUNING_MODE, Integer value, Integer defaultValue) {
    Logger.recordOutput(
        key, TUNING_MODE ? SmartDashboard.getNumber(key, defaultValue) : defaultValue);
  }

  @Override
  public int getAsInt() {
    return get();
  }
}
