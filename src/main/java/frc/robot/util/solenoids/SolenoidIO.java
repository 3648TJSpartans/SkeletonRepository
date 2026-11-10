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

package frc.robot.util.solenoids;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Status;
import frc.robot.util.statusableUtils.Statusable;
import org.littletonrobotics.junction.Logger;

public abstract class SolenoidIO extends SubsystemBase implements Statusable {
  private final String m_name;

  public SolenoidIO(String name) {
    m_name = name;
  }

  public final String getName() {
    return m_name;
  }

  public abstract void setSolenoid(boolean on);

  public abstract void toggleSolenoid();

  public abstract boolean getSolenoidOn();

  public abstract boolean getSolenoidEnabled();

  public void updateValues() {
    Logger.recordOutput(m_name + "/getSolenoidOn", getSolenoidOn());
    Logger.recordOutput(m_name + "/getSolenoidEnabled", getSolenoidEnabled());
  }

  @Override
  public void periodic() {
    updateValues();
  }

  public abstract Status getStatus();
}
