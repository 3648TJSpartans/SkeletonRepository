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

package frc.robot.commands.exampleSubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.exampleMotorSubsystem.ExampleMotorSubsystem;
import org.littletonrobotics.junction.Logger;

public class ExampleMotorCmd extends Command {
  private final ExampleMotorSubsystem m_exampleMotorSubsystem;
  private final double power;

  public ExampleMotorCmd(ExampleMotorSubsystem exampleMotorSubsystem, double power) {
    m_exampleMotorSubsystem = exampleMotorSubsystem;
    this.power = power;
    addRequirements(m_exampleMotorSubsystem);
  }

  @Override
  public void initialize() {
    Logger.recordOutput("exampleSubsystemCommand/CommandScheduled", true);
  }

  @Override
  public void execute() {
    Logger.recordOutput("exampleSubsystemCommand/Speed", power);
    m_exampleMotorSubsystem.setPower(power);
  }

  @Override
  public void end(boolean interrupted) {
    m_exampleMotorSubsystem.stop();
    Logger.recordOutput("exampleSubsystemCommand/CommandScheduled", false);
  }

  @Override
  public boolean isFinished() {
    /*
     * Because this command lacks a finish state, it's important to make sure it ends some other
     * way, such as using whileTrue (which will make the command end when the button is
     * released) or a raceGroup, which finishes all commands in the group when any command in
     * the group finishes.
     */
    return false;
  }
}
