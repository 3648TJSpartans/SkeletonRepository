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

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.util.Tunable;
import frc.robot.util.TuningUpdater;
import org.littletonrobotics.junction.Logger;

public class TunableTransform3d extends Tunable<Transform3d> {
  private TunableTranslation3d translation;
  private TunableRotation3d rotation;

  public TunableTransform3d(String dashboardKey) {
    super(dashboardKey);
  }

  public TunableTransform3d(String dashboardKey, Transform3d defaultTransform3d) {
    super(dashboardKey, defaultTransform3d);
  }

  public TunableTransform3d(String dashboardKey, Transform3d defaultTransform3d, Runnable update) {
    super(dashboardKey, defaultTransform3d, update);
  }

  @Override
  protected void putDashboardValue(String key, Transform3d defaultValue) {
    key =
        key.substring(
            TuningUpdater.TABLE_KEY.length() + 1); // Makes sure the TableKey isn't added twice.
    translation = new TunableTranslation3d(key + "/translation", defaultValue.getTranslation());
    rotation = new TunableRotation3d(key + "/rotation", defaultValue.getRotation());
  }

  @Override
  protected Transform3d getDashboardValue(String key, Transform3d defaultValue) {
    return new Transform3d(
        translation.getDashboardValue(key + "/translation", defaultValue.getTranslation()),
        rotation.getDashboardValue(key + "/rotation", defaultValue.getRotation()));
  }

  @Override
  protected void logValue(
      String key, boolean TUNING_MODE, Transform3d value, Transform3d defaultValue) {
    // TODO Auto-generated method stub
    Logger.recordOutput(key, getDashboardValue(key, defaultValue));
  }
}
