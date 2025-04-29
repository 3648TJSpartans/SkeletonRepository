package frc.robot.subsystems.relativeEncoder;

public interface RelEncoderIO {
    // allows us to move to a certain point
    public default void setTo(double setpoint) {
    }

    public default void updateValues() {
    }

    public default boolean getLimitSwitch() {
        return false;
    }

    public default void stop() {
    }

    public default double getPos() {
        return 0;
    }

    public default void resetEncoder() {
    }

    public default void setSpeed(double speed) {
    }

    public default void updateLimitSwitch() {
    }

    public default boolean atZero() {
        return true;
    }

    public default boolean getLimitReset() {
        return true;
    }
}
