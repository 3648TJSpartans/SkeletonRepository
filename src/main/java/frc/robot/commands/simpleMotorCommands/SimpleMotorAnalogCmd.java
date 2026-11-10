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

package frc.robot.commands.simpleMotorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.simpleMotor.SimpleMotor;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class SimpleMotorAnalogCmd extends Command {
  private final SimpleMotor m_simpleMotor;
  private final Supplier<Double> speed;

  public SimpleMotorAnalogCmd(SimpleMotor simpleMotor, Supplier<Double> speed) {
    m_simpleMotor = simpleMotor;
    this.speed = speed;
    addRequirements(m_simpleMotor);
  }

  @Override
  public void initialize() {
    Logger.recordOutput("simpleMotor/AnalogCommand/Scheduled", true);
  }

  @Override
  public void execute() {
    Logger.recordOutput("simpleMotor/AnalogCommand/Speed", speed.get());
    m_simpleMotor.setSpeed(speed.get());
  }

  @Override
  public void end(boolean interrupted) {
    m_simpleMotor.stop();
    Logger.recordOutput("simpleMotor/AnalogCommand/Scheduled", false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
