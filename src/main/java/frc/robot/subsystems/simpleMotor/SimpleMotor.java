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

package frc.robot.subsystems.simpleMotor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/*
 * This subsystem controls a single motor with commands to spin
 * or stop it. Importantly, it lacks an encoder, which means that the
 * robot won't know how much the motor has spun or its current position.
 */

/*
 * Subsystems are split into a main file, an IO, and a third file which houses
 * most of the logic. This file, the main file, receives an IO, and uses it
 * to structure the subsystem.
 */

public class SimpleMotor extends SubsystemBase {

  private final SimpleMotorIO io;

  public SimpleMotor(SimpleMotorIO io) {
    this.io = io;
  }

  /* Periodic, which runs constantly, is used here to log important values. */
  @Override
  public void periodic() {
    Logger.recordOutput("simpleMotor/speed", io.getSpeed());
  }

  /* The remaining methods are all taken from the IO. */

  public void stop() {
    io.stop();
  }

  public void setSpeed(double speed) {
    io.setSpeed(speed);
  }

  public double getSpeed(double speed) {
    return io.getSpeed();
  }
}
