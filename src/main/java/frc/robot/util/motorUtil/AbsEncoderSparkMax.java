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

    public AbsEncoderSparkMax(String loggingName, TunableNumber motorCan, double positionTolerance,
            double speedTolerance, double kP, double kI, double kD, double kFF, double minPower,
            double maxPower, double encoderOdometryFrequency) {
        super(loggingName, positionTolerance, speedTolerance);
        motor = new SparkMax((int) motorCan.get(), MotorType.kBrushless);
        motorController = motor.getClosedLoopController();
        encoder = motor.getAbsoluteEncoder();
        name = getName();
        var config = new SparkMaxConfig();
        config.inverted(false).idleMode(IdleMode.kBrake).voltageCompensation(12.0);
        config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pidf(new TunableNumber(name + "/PIDF/P", kP).get(),
                        new TunableNumber(name + "/PIDF/I", kI).get(),
                        new TunableNumber(name + "/PIDF/D", kD).get(),
                        new TunableNumber(name + "/PIDF/FF", kFF).get())
                .outputRange(new TunableNumber(name + "/PowerRange/minPower", minPower).get(),
                        new TunableNumber(name + "/PowerRange/maxPower", maxPower).get());
        config.signals.absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs((int) (1000.0 / encoderOdometryFrequency))
                .absoluteEncoderVelocityAlwaysOn(true).absoluteEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    @Override
    public void setPosition(double setpoint) {
        motorController.setReference(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void setSpeed(double speed) {
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
}
