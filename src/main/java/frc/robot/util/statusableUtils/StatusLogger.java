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

package frc.robot.util.statusableUtils;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Status;
import org.littletonrobotics.junction.Logger;

public class StatusLogger extends SubsystemBase {

  Statusable[] m_statuses;

  public StatusLogger(Statusable... statuses) {
    m_statuses = statuses;
  }

  public void logStatuses() {
    for (Statusable status : m_statuses) {
      Logger.recordOutput("Utils/Statusable/" + status.getName(), status.getStatus());
    }
  }

  public Statusable[] getStatuses() {
    if (m_statuses == null) {
      return new Statusable[] {
        new Statusable() {
          @Override
          public Status getStatus() {
            return Status.UNKNOWN;
          }

          @Override
          public String getName() {
            return "Default Status";
          }
        }
      };
    }
    return m_statuses;
  }
}
