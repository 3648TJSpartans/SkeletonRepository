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

import static frc.robot.util.TuningUpdater.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

public class TunableBoolean extends Tunable<Boolean> implements BooleanSupplier {

  /**
   * Create a new TunableNumber
   *
   * @param dashboardKey Key on dashboard
   */
  public TunableBoolean(String dashboardKey) {
    super(dashboardKey);
  }

  /**
   * Create a new TunableNumber with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public TunableBoolean(String dashboardKey, Boolean defaultValue) {
    super(dashboardKey, defaultValue);
  }

  public TunableBoolean(String dashboardKey, Boolean defaultValue, Runnable update) {
    super(dashboardKey, defaultValue, update);
  }

  public TunableBoolean(
      String dashboardKey, Boolean defaultValue, Consumer<Boolean> updateConsumer) {
    super(dashboardKey, defaultValue, updateConsumer);
  }

  public TunableBoolean(
      String dashboardKey,
      Boolean defaultValue,
      Runnable update,
      Consumer<Boolean> updateConsumer) {
    super(dashboardKey, defaultValue, update, updateConsumer);
  }

  public boolean getAsBoolean() {
    return get();
  }

  @Override
  protected void putDashboardValue(String key, Boolean defaultValue) {
    SmartDashboard.putBoolean(key, SmartDashboard.getBoolean(key, defaultValue));
  }

  @Override
  protected Boolean getDashboardValue(String key, Boolean defaultValue) {
    return TUNING_MODE ? SmartDashboard.getBoolean(key, defaultValue) : defaultValue;
  }

  @Override
  protected void logValue(String key, boolean TUNING_MODE, Boolean value, Boolean defaultValue) {
    Logger.recordOutput(
        key, TUNING_MODE ? SmartDashboard.getBoolean(key, defaultValue) : defaultValue);
  }
}
