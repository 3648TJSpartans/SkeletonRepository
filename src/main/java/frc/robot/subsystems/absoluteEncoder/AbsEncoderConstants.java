package frc.robot.subsystems.absoluteEncoder;

import frc.robot.util.TunableNumber;

public class AbsEncoderConstants {
    public static final int absEncoderMotorCan = 15;

    public static final TunableNumber kP = new TunableNumber("absEncoder/kP", 0);
    public static final TunableNumber kI = new TunableNumber("absEncoder/kI", 0);
    public static final TunableNumber kD = new TunableNumber("absEncoder/kD", 0);
    public static final TunableNumber kFF = new TunableNumber("absEncoder/kFF", 0);

    public static final TunableNumber kMinRange = new TunableNumber("absEncoder/kMinRange", -1);
    public static final TunableNumber kMaxRange = new TunableNumber("absEncoder/kMaxRange", 1);

    public static final TunableNumber setpoint1 = new TunableNumber("absEncoder/setpoint1", 0);
    public static final TunableNumber setpoint2 = new TunableNumber("absEncoder/setpoint2", 0.5);

    public static final TunableNumber setpointTolerance =
            new TunableNumber("absEncoder/setpointTolerance", 0.05);

    public static TunableNumber kAbsEncoderOdometryFrequency =
            new TunableNumber("absEncoder/kAbsEncoderOdometryFrequency", 100);
    public static final boolean encoderInverted = false;
    public static final double encoderPositionFactor = 1;

}
