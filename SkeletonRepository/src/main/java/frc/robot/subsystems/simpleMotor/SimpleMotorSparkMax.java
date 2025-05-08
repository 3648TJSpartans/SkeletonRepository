package frc.robot.subsystems.simpleMotor;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

/*
 * This is the file where most of the logic is applied. It
 * implements the IO, which means it must use the same methods
 * that the IO gives it.
 */

public class SimpleMotorSparkMax implements SimpleMotorIO {

    private SparkMax motor;

    /* The constructor defines the motor and any necessary variables. */
    public SimpleMotorSparkMax() {
        /* All of our motors are brushless. */
        motor = new SparkMax(SimpleMotorConstants.simpleMotorCan, MotorType.kBrushless);
    }

    /* Any necessary logging is performed here. */
    public void updateValues() {
        Logger.recordOutput("simpleMotor/speed", getSpeed());
    }

    public void stop() {
        motor.set(0);
    }

    public void setSpeed(double speed) {
        motor.set(speed);
    }

    public double getSpeed() {
        return motor.get();
    }
}
