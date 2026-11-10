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

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

/*
 * This is the file where most of the logic is applied. It
 * implements the IO, which means it must use the same methods
 * that the IO gives it.
 */

public class SimpleMotorSparkMax implements SimpleMotorIO {

  private SparkMax motor;

  /* The constructor defines the motor and any necessary variables. */
  public SimpleMotorSparkMax() {
    /* All of our motors are brushless. */
    motor = new SparkMax(SimpleMotorConstants.simpleMotorCan, MotorType.kBrushless);
  }

  public void stop() {
    motor.set(0);
  }

  public void setSpeed(double speed) {
    motor.set(speed);
  }

  public double getSpeed() {
    return motor.get();
  }
}
