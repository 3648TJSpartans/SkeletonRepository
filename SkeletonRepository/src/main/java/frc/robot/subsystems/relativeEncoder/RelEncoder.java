// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.relativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RelEncoder extends SubsystemBase {
  // declaration of a instance
  private final RelEncoderIO io;

  public RelEncoder(RelEncoderIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateValues();
    io.updateLimitSwitch();
  }

  public boolean getLimitSwitch() {
    return io.getLimitSwitch();
  }

  public void setTo(double setpoint) {
    io.setTo(setpoint);
  }

  // allows us to stop the motor
  public void stop() {
    io.stop();
  }

  public double getPosition() {
    return io.getPosition();
  }

  // allows us to reset the encoder value
  public void resetEncoder() {
    io.resetEncoder();
  }

  public void setSpeed(double speed) {
    io.setSpeed(speed);
  }

  public boolean atZero() {
    return io.atZero();
  }

  public boolean getLimitReset() {
    return io.getLimitReset();
  }
}
