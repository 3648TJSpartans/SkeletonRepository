package frc.robot.subsystems.simpleMotor;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.util.TunableNumber;

public class SimpleMotorSparkMax implements SimpleMotorIO {

    private SparkMax motor;

    public SimpleMotorSparkMax() {
        motor = new SparkMax(SimpleMotorConstants.simpleMotorCan, MotorType.kBrushless);
    }

    @Override
    public void stop() {
        motor.set(0);
    }

    public void setSpeed(double speed) {
        motor.set(speed);
    }

    @Override
    public double getSpeed() {
        return motor.get();
    }
}
