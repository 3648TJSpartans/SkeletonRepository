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

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants.Status;
import org.littletonrobotics.junction.Logger;

public class SingleSolenoid extends SolenoidIO {

  private Solenoid m_solenoid;

  public SingleSolenoid(int channel, String name) {
    super(name);
    m_solenoid = new Solenoid(PneumaticsModuleType.REVPH, channel);
  }

  public void setSolenoid(boolean on) {
    m_solenoid.set(on);
  }

  public void toggleSolenoid() {
    m_solenoid.toggle();
  }

  public boolean getSolenoidOn() {
    return m_solenoid.get();
  }

  public boolean getSolenoidEnabled() {
    return !m_solenoid.isDisabled();
  }

  public Status getStatus() {
    Status localStatus = Status.OK;
    if (!getSolenoidEnabled()) {
      localStatus = Status.WARNING;
      Logger.recordOutput("Debug/Subsystems/" + getName() + "/warning", "Solenoid Disabled");
    }
    return localStatus;
  }
}
