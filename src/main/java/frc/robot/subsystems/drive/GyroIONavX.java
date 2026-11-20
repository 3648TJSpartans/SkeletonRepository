// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
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
//
// Modified by FRC Team 3648 to add and change functionality as needed for our robot.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.RadiansPerSecond;

import com.studica.frc.Navx;
import java.util.Queue;
import org.wpilib.math.geometry.Rotation2d;

/** IO implementation for NavX. */
public class GyroIONavX implements GyroIO {

  private final Navx navX = Navx.usb(DriveConstants.navXComType, (int) odometryFrequency);

  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  public GyroIONavX() {
    yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue =
        SparkOdometryThread.getInstance()
            .registerSignal(() -> navX.getAngle().map(a -> a.in(Degrees)).orElse(0.0));
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = navX.isConnected();
    inputs.yawPosition =
        Rotation2d.fromDegrees(-navX.getAngle().map(a -> a.in(Degrees)).orElse(0.0));
    inputs.yawVelocityRadPerSec =
        -navX.getAngularVelocity().map(v -> v.z().in(RadiansPerSecond)).orElse(0.0);
    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(-value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
