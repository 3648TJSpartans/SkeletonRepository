package frc.robot.subsystems.exampleMotorSubsystem;

import frc.robot.util.TunableNumber;
import frc.robot.util.motorUtil.MotorConfig;

/*
 * Constants files contain numbers and other values that shouldn't change while robot code is
 * running, except for tuning.
 */

public class ExampleMotorSubsystemConstants {

    public static final int motor1Can = 15;
    public static final int motor2Can = 16;

    public static final double kP = 0.5;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kFF = 0.0;
    public static final double kOdometryFrequency = 100;
    public static final double kMinRange = -0.1;
    public static final double kMaxRange = 0.1;
    public static final boolean encoderInverted = false;
    public static final double elevatorEncoderPositionFactor = 1;
    public static final double elevatorEncoderVelocityFactor = 1;

    public static final int limitSwitchPin = 1;

    public static final double positionTolerance = 0.05;
    public static final double speedTolerance = 0.05;

    public static final MotorConfig motor1Config =
            new MotorConfig("exampleSubsystem/motor1").motorCan(motor1Can).p(kP).i(kI).d(kD).ff(kFF)
                    .encoderOdometryFrequency(kOdometryFrequency).minPower(kMinRange)
                    .maxPower(kMaxRange).isInverted(encoderInverted)
                    .positionTolerance(positionTolerance).speedTolerance(speedTolerance);
    // Copies motor 1 config except for changing name and CAN id
    public static final MotorConfig motor2Config =
            motor1Config.name("exampleSubsystem/motor2").motorCan(motor2Can);
}
