package frc.robot.subsystems.relativeEncoder;

import frc.robot.util.TunableNumber;

/*
 * Constants files contain numbers and other values that shouldn't change while robot code is
 * running, except for tuning.
 */

public class RelEncoderConstants {

    public static final int relEncoderMotorCan = 14;

    public static final TunableNumber setpoint1 = new TunableNumber("relEncoder/setpoint1", 0);
    public static final TunableNumber setpoint2 = new TunableNumber("relEncoder/setpoint2", 1);
    public static final TunableNumber setpointTolerance =
            new TunableNumber("relEncoder/setpointTolerance", 0);
    public static final TunableNumber relEncoderLimit =
            new TunableNumber("relEncoder/relEncoderLimit", 2);

    public static final TunableNumber kP = new TunableNumber("relEncoder/kP", 0);
    public static final TunableNumber kI = new TunableNumber("relEncoder/kI", 0);
    public static final TunableNumber kD = new TunableNumber("relEncoder/kD", 0);
    public static final TunableNumber kFF = new TunableNumber("relEncoder/kFF", 0);
    public static final double odometryFrequency = 100;
    public static final TunableNumber kMinRange = new TunableNumber("relEncoder/kMinRange", -1);
    public static final TunableNumber kMaxRange = new TunableNumber("relEncoder/kMaxRange", 1);
    public static final boolean encoderInverted = false;
    public static final TunableNumber elevatorEncoderPositionFactor =
            new TunableNumber("relEncoder/elevatorEncoderPositionFactor", 1);
    public static final TunableNumber elevatorEncoderVelocityFactor =
            new TunableNumber("relEncoder/elevatorEncoderVelocityFactor", 1);

    public static final int limitSwitchPin = 1;

}
