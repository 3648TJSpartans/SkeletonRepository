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

import frc.robot.Constants.Status;
import org.littletonrobotics.junction.Logger;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.pneumatic.DoubleSolenoid;
import org.wpilib.hardware.pneumatic.DoubleSolenoid.Value;
import org.wpilib.hardware.pneumatic.PneumaticsModuleType;

public class DoubleSolenoidIO extends SolenoidIO {

  private DoubleSolenoid m_solenoid;

  public DoubleSolenoidIO(int solenoidChannel1, int solenoidChannel2, String name) {
    super(name);
    m_solenoid =
        new DoubleSolenoid(
            CANPort.CAN_S0, PneumaticsModuleType.REV_PH, solenoidChannel1, solenoidChannel2);
  }

  public void setSolenoidForward() {
    m_solenoid.set(DoubleSolenoid.Value.FORWARD);
  }

  public void setSolenoidReverse() {
    m_solenoid.set(DoubleSolenoid.Value.REVERSE);
  }

  public void setSolenoidOff() {
    m_solenoid.set(DoubleSolenoid.Value.OFF);
  }

  public void toggleSolenoid() {
    m_solenoid.toggle();
  }

  public boolean getForwardSolenoidOn() {
    return m_solenoid.get() == Value.FORWARD;
  }

  public boolean getReverseSolenoidOn() {
    return m_solenoid.get() == Value.REVERSE;
  }

  public boolean getForwardSolenoidEnabled() {
    return !m_solenoid.isFwdSolenoidDisabled();
  }

  public boolean getReverseSolenoidEnabled() {
    return !m_solenoid.isRevSolenoidDisabled();
  }

  // Returns WARNING because getStatus() has not been configured (this method should be overwritten)
  public Status getStatus() {
    return Status.WARNING;
  }

  @Override
  public void setSolenoid(boolean on) {
    // TODO Auto-generated method stub
    if (on) {
      setSolenoidForward();
      Logger.recordOutput(getName() + "/state", "forward");
      return;
    }
    Logger.recordOutput(getName() + "/state", "reverse");
    setSolenoidReverse();
  }

  @Override
  public boolean getSolenoidOn() {
    return getForwardSolenoidEnabled();
  }

  @Override
  public boolean getSolenoidEnabled() {
    return getForwardSolenoidEnabled() || getReverseSolenoidEnabled();
  }
}
