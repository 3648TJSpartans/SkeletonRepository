package frc.robot.util.motorUtil;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.util.TunableNumber;

import com.revrobotics.spark.SparkClosedLoopController;

public class AbsEncoderSparkMax extends MotorIO {

    private SparkMax motor;
    private AbsoluteEncoder encoder;
    private SparkClosedLoopController motorController;
    private String name;

    public AbsEncoderSparkMax(MotorConfig motorConfig) {
        super(motorConfig.name(), motorConfig.positionTolerance(), motorConfig.speedTolerance());
        motor = new SparkMax(motorConfig.motorCan(), MotorType.kBrushless);
        motorController = motor.getClosedLoopController();
        encoder = motor.getAbsoluteEncoder();
        name = getName();
        var config = new SparkMaxConfig();
        config.inverted(false).idleMode(IdleMode.kBrake).voltageCompensation(12.0);
        config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pidf(motorConfig.p(), motorConfig.i(), motorConfig.d(), motorConfig.ff())
                .outputRange(motorConfig.minPower(), motorConfig.maxPower());
        config.signals.absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs(
                        (int) (1000.0 / motorConfig.encoderOdometryFrequency()))
                .absoluteEncoderVelocityAlwaysOn(true).absoluteEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    @Override
    public void setPosition(double setpoint) {
        super.setPosition(setpoint);
        motorController.setReference(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void setSpeed(double speed) {
        super.setSpeed(speed);
        motorController.setReference(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void setPower(double power) {
        motor.set(power);
    }

    @Override
    public double getPosition() {
        return encoder.getPosition();
    }

    @Override
    public double getSpeed() {
        return encoder.getVelocity();
    }

    // Absolute Encoders cannot reset
    @Override
    public void setEncoder(double setpoint) {
        return;
    }
}
