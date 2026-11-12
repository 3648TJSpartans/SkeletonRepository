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

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.PoseConstants;

public class RangeCalc {

  /*
   * zone 0 is right,
   * zone 1 middle,
   * zone 2 left
   */
  public static int zoneCalc(Pose2d robotPose) {
    double x = AllianceFlipUtil.applyY(robotPose.getY());
    if (x < 4.021) {
      return 0;
    }
    return 1;
  }

  public static boolean inField(Pose2d robotPose) {
    double x = robotPose.getX();
    if (x < 0.3429) {
      return false;
    }
    if (x > PoseConstants.fieldLength - 0.3429) {
      return false;
    }
    double y = robotPose.getY();
    if (y < 0.3429) {
      return false;
    }
    if (y > PoseConstants.fieldWidth - 0.3429) {
      return false;
    }

    return true;
  }
}
