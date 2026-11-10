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

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.util.Tunable;
import frc.robot.util.TuningUpdater;
import org.littletonrobotics.junction.Logger;

public class TunablePose3d extends Tunable<Pose3d> {
  private TunableTranslation3d translation;
  private TunableRotation3d rotation;

  public TunablePose3d(String dashboardKey) {
    super(dashboardKey);
  }

  public TunablePose3d(String dashboardKey, Pose3d defaultPose) {
    super(dashboardKey, defaultPose);
  }

  public TunablePose3d(String dashboardKey, Pose3d defaultPose, Runnable update) {
    super(dashboardKey, defaultPose, update);
  }

  @Override
  protected void putDashboardValue(String key, Pose3d defaultValue) {
    key =
        key.substring(
            TuningUpdater.TABLE_KEY.length() + 1); // Makes sure the TableKey isn't added twice.
    translation = new TunableTranslation3d(key + "/translation", defaultValue.getTranslation());
    rotation = new TunableRotation3d(key + "/rotation", defaultValue.getRotation());
  }

  @Override
  protected Pose3d getDashboardValue(String key, Pose3d defaultValue) {
    return new Pose3d(
        translation.getDashboardValue(key + "/translation", defaultValue.getTranslation()),
        rotation.getDashboardValue(key + "/rotation", defaultValue.getRotation()));
  }

  @Override
  protected void logValue(String key, boolean TUNING_MODE, Pose3d value, Pose3d defaultValue) {
    // TODO Auto-generated method stub
    Logger.recordOutput(key, getDashboardValue(key, defaultValue));
  }
}
