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

package frc.robot.util.motorUtil;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MultiMotorSubsystem extends SubsystemBase {
  private final MotorIO[] m_motors;

  public MultiMotorSubsystem(MotorIO... motors) {
    m_motors = motors;
  }

  public void setPosition(double position) {
    for (MotorIO motor : m_motors) {
      motor.setPosition(position);
    }
  }

  public void setSpeed(double speed) {
    for (MotorIO motor : m_motors) {
      motor.setSpeed(speed);
    }
  }

  public void setPower(double power) {
    for (MotorIO motor : m_motors) {
      motor.setPower(power);
    }
  }

  public void setEncoder(double setpoint) {
    for (MotorIO motor : m_motors) {
      motor.setEncoder(setpoint);
    }
  }

  public final void resetEncoder() {
    for (MotorIO motor : m_motors) {
      motor.resetEncoder();
    }
  }

  public final void stop() {
    for (MotorIO motor : m_motors) {
      motor.stop();
    }
  }
}
