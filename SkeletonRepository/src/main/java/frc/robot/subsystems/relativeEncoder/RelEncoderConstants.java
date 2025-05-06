// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.relativeEncoder;

/** constants for the coral subsystem */
public class RelEncoderConstants {

    public static final int relEncoderMotorCan = 9;

    public static final double setpoint1 = 0;
    public static final double setpoint2 = 1;
    public static final double relativeEncoderLimit = 2;
    public static final double elevatorRange = 0.622;
    public static final double heightToEncoderRatio = elevatorRange / relativeEncoderLimit;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kFF = 0;
    public static final double odometryFrequency = 100;
    public static final double kMinRange = -1;
    public static final double kMaxRange = 1;
    public static final boolean encoderInverted = false;
    public static final double elevatorEncoderPositionFactor = 1;
    public static final double elevatorEncoderVelocityFactor = 1;

    public static final int limitSwitchPin = 1;

    public static final double marginOfError = 1;

}
