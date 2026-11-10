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
import edu.wpi.first.wpilibj.simulation.SolenoidSim;
import frc.robot.Constants.Status;

public class SingleSolenoidSim extends SolenoidIO {

  private SolenoidSim m_solenoid;
  private boolean on;

  public SingleSolenoidSim(int channel, String name) {
    super(name);
    m_solenoid = new SolenoidSim(PneumaticsModuleType.REVPH, channel);
    on = false;
  }

  public void setSolenoid(boolean on) {
    this.on = on;
  }

  public void toggleSolenoid() {
    on = !on;
  }

  public boolean getSolenoidOn() {
    return on;
  }

  public boolean getSolenoidEnabled() {
    return true;
  }

  public Status getStatus() {
    return Status.UNKNOWN;
  }
}
