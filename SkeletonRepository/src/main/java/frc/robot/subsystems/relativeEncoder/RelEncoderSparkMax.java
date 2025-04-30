// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.relativeEncoder;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;

import frc.robot.util.TunableNumber;

public class RelEncoderSparkMax implements RelEncoderIO {

  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController motorController;
  private boolean limitReset;
  private final DigitalInput limitSwitch = new DigitalInput(RelEncoderConstants.limitSwitchPin);

  public RelEncoderSparkMax() {
    motor = new SparkMax(RelEncoderConstants.relEncoderMotorCan, MotorType.kBrushless);
    motorController = motor.getClosedLoopController();
    limitReset = false;
    Logger.recordOutput("relEncoder/EncoderReset", false);
    Logger.recordOutput("relEncoder/Setpoint", 0.0);
    encoder = motor.getEncoder();

    var motorConfig = new SparkMaxConfig();
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.inverted(false)
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12.0);
    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(new TunableNumber("relEncoder/kP", RelEncoderConstants.kP).get(),
            new TunableNumber("relEncoder/kI", RelEncoderConstants.kI).get(),
            new TunableNumber("relEncoder/kD", RelEncoderConstants.kD).get(),
            new TunableNumber("relEncoder/kFF", RelEncoderConstants.kFF).get())
        .outputRange(
            new TunableNumber("relEncoder/kMinRange", RelEncoderConstants.kMinRange)
                .get(),
            new TunableNumber("relEncoder/kMaxRange", RelEncoderConstants.kMaxRange)
                .get());
    motorConfig.signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs((int) (1000.0 / RelEncoderConstants.odometryFrequency))
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    motorConfig.absoluteEncoder
        .inverted(false)
        .positionConversionFactor(new TunableNumber("relEncoder/encoderPositionFactor",
            RelEncoderConstants.elevatorEncoderPositionFactor).get())
        .velocityConversionFactor(RelEncoderConstants.elevatorEncoderPositionFactor)
        .averageDepth(2);
    motor.configure(
        motorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void stop() {
    motor.set(0);
  }

  @Override
  public double getPosition() {
    return encoder.getPosition();
  }

  @Override
  public void setTo(double setpoint) {
    Logger.recordOutput("relEncoder/Setpoint", setpoint);

    if (!limitReset) {
      Logger.recordOutput("relEncoder/NotWorkingBecause", "NotHomed");
      motor.set(0);
      return;
    }
    if (getLimitSwitch() && setpoint - getPosition() < 0) {
      motorController.setReference(getPosition(), ControlType.kPosition, ClosedLoopSlot.kSlot0);
      Logger.recordOutput("relEncoder/NotWorkingBecause", "TooLittle");
      return;
    }

    motorController.setReference(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    Logger.recordOutput("relEncoder/NotWorkingBecause", "Working");
  }

  public void setSpeed(double speed) {
    motor.set(speed);
  }

  @Override
  public void updateValues() {
    Logger.recordOutput("relEncoder/EncoderValue", encoder.getPosition());
  }

  @Override
  public void updateLimitSwitch() {
    if (getLimitSwitch()) {
      setZero();
    }
  }

  @Override
  @AutoLogOutput(key = "relEncoder/limitSwitchPushed")
  public boolean getLimitSwitch() {
    return !limitSwitch.get();
  }

  private void setZero() {
    encoder.setPosition(0);
    limitReset = true;
    Logger.recordOutput("relEncoder/EncoderReset", limitReset);
  }

  @Override
  public boolean atZero() {
    return !limitSwitch.get();
  }

  @Override
  public boolean getLimitReset() {
    return limitReset;
  }

}
