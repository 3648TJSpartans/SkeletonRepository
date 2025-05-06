package frc.robot.subsystems.simpleMotor;

public interface SimpleMotorIO {

    public default void stop() {

    }

    public default void setSpeed(double speed) {

    }

    public default double getSpeed() {
        return 0;
    }
}
