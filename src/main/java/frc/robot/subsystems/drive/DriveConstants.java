// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
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

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.util.TunableNumber;

public class DriveConstants {
        public static final TunableNumber fieldRelativeMaxInputPercent =
                        new TunableNumber("drive/fieldRelativeMaxInputPercent", 1);
        // value between 0+ and 1,
        // allowing control maximum
        // input for field relative
        public static final TunableNumber robotRelativeMaxInputPercent =
                        new TunableNumber("drive/robotRelativeMaxInputPercent", .5);
        // value between 0+ and 1,
        // allowing control
        // maximum input for robot
        // relative

        public static final TunableNumber maxSpeedMetersPerSec =
                        new TunableNumber("drive/maxSpeedMetersPerSec", 4);
        public static final double odometryFrequency = 100.0; // Hz
        public static final double trackWidth = .6;
        public static final double wheelBase = .6;
        public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
        public static final Translation2d[] moduleTranslations =
                        new Translation2d[] {new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
                                        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
                                        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
                                        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)};

        // Zeroed rotation values for each module, see setup instructions
        public static final Rotation2d frontLeftZeroRotation = new Rotation2d(-Math.PI / 2);
        public static final Rotation2d frontRightZeroRotation = new Rotation2d(0);
        public static final Rotation2d backLeftZeroRotation = new Rotation2d(Math.PI);
        public static final Rotation2d backRightZeroRotation = new Rotation2d(Math.PI / 2);

        // Device CAN IDs
        public static final int pigeonCanId = 9;

        public static final int frontLeftDriveCanId = 8;
        public static final int backLeftDriveCanId = 6;
        public static final int frontRightDriveCanId = 2;
        public static final int backRightDriveCanId = 4;

        public static final int frontLeftTurnCanId = 7;
        public static final int backLeftTurnCanId = 5;
        public static final int frontRightTurnCanId = 1;
        public static final int backRightTurnCanId = 3;

        // Drive motor configuration
        public static final int driveMotorCurrentLimit = 50;
        public static final double wheelRadiusMeters = Units.inchesToMeters(1.458);
        public static final double driveMotorReduction = (45.0 * 22.0) / (13.0 * 15.0); // MAXSwerve
                                                                                        // with 14
                                                                                        // pinion
                                                                                        // teeth
                                                                                        // and 22
                                                                                        // spur
                                                                                        // teeth
        public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);

        // Drive encoder configuration
        public static final double driveEncoderPositionFactor = 2 * Math.PI / driveMotorReduction; // Rotor
                                                                                                   // Rotations
                                                                                                   // ->
                                                                                                   // Wheel
                                                                                                   // Radians
        public static final double driveEncoderVelocityFactor =
                        (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM
                                                                    // ->
                                                                    // Wheel
                                                                    // Rad/Sec

        // Drive PID configuration
        public static final TunableNumber driveKp = new TunableNumber("drive/driveKp", 0);
        public static final TunableNumber driveKd = new TunableNumber("drive/driveKd", 0);
        public static final TunableNumber driveKs = new TunableNumber("drive/driveKs", 0.14381);
        public static final TunableNumber driveKv = new TunableNumber("drive/driveKv", 0.10033);
        public static final TunableNumber driveSimP = new TunableNumber("drive/driveSimP", 0.05);
        public static final TunableNumber driveSimD = new TunableNumber("drive/driveSimD", 0);
        public static final TunableNumber driveSimKs = new TunableNumber("drive/driveSimKs", 0);
        public static final TunableNumber driveSimKv =
                        new TunableNumber("drive/driveSimKv", 0.0789);

        // Turn motor configuration
        public static final boolean turnInverted = false;
        public static final int turnMotorCurrentLimit = 20;
        public static final double turnMotorReduction = 9424.0 / 203.0;
        public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

        // Turn encoder configuration
        public static final boolean turnEncoderInverted = true;
        public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
        public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM ->
                                                                                     // Rad/Sec

        // Turn PID configuration
        public static final TunableNumber turnKp = new TunableNumber("drive/turnKp", 2.0);
        public static final TunableNumber turnKd = new TunableNumber("drive/turnKd", 0);
        public static final TunableNumber turnSimP = new TunableNumber("drive/turnSimP", 8.0);
        public static final TunableNumber turnSimD = new TunableNumber("drive/turnSimD", 0);
        public static final double turnPIDMinInput = 0; // Radians
        public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

        // PathPlanner configuration
        public static final double robotMassKg = 45.3;
        public static final double robotMOI = 6.883;
        public static final double wheelCOF = 1.2;
        public static final double ppMaxSpeed = 4.2;
        public static final RobotConfig ppConfig = new RobotConfig(robotMassKg, robotMOI,
                        new ModuleConfig(wheelRadiusMeters, ppMaxSpeed, wheelCOF,
                                        driveGearbox.withReduction(driveMotorReduction),
                                        driveMotorCurrentLimit, 1),
                        moduleTranslations);
}
