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

/*
 * The IO file creates a skeleton for the third file with
 * most of the logic. If a subsystem could have two variants,
 * such as one with one motor and one with two, both could
 * run through the same IO structure.
 */

public interface SimpleMotorIO {

  public default void stop() {}

  public default void setSpeed(double speed) {}

  public default double getSpeed() {
    return 0;
  }
}
