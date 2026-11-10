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

package frc.robot.subsystems.exampleMotorSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.motorUtil.MotorIO;

public class ExampleMotorSubsystem extends SubsystemBase {
  private final MotorIO m_motor1;
  private final MotorIO m_motor2;

  public ExampleMotorSubsystem() {
    m_motor1 = ExampleMotorSubsystemConstants.motor1;
    m_motor2 = ExampleMotorSubsystemConstants.motor2;
  }

  public void setPower(double power) {
    m_motor1.setPower(power);
    m_motor2.setPower(power);
  }

  public void stop() {
    m_motor1.stop();
    m_motor2.stop();
  }
}
