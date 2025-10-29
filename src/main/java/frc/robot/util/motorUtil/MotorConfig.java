package frc.robot.util.motorUtil;

import frc.robot.util.TunableNumber;

public class MotorConfig {
    private String m_loggingName = "defaultMotor";
    private int m_motorCan = 1;
    private double m_positionTolerance = 0.0;
    private double m_speedTolerance = 0.0;
    private double m_P = 0.0;
    private double m_I = 0.0;
    private double m_D = 0.0;
    private double m_FF = 0.0;
    private double m_minPower = 0.0;
    private double m_maxPower = 0.0;
    private double m_encoderOdometryFrequency = 100;
    private boolean m_isInverted = false;

    public MotorConfig(String name) {
        m_loggingName = "MotorIOs/" + name;
    }

    public MotorConfig name(String name) {
        m_loggingName = "MotorIOs/" + name;
        return this;
    }

    public MotorConfig motorCan(int motorCAN) {
        m_motorCan = (int) new TunableNumber(m_loggingName + "/motorCAN", motorCAN).get();
        return this;
    }

    public MotorConfig positionTolerance(double positionTolerance) {
        m_positionTolerance = positionTolerance;
        return this;
    }

    public MotorConfig speedTolerance(double speedTolerance) {
        m_speedTolerance = speedTolerance;
        return this;
    }

    public MotorConfig p(double p) {
        m_P = new TunableNumber(m_loggingName + "/PIDF/P", p).get();
        return this;
    }

    public MotorConfig i(double i) {
        m_I = new TunableNumber(m_loggingName + "/PIDF/I", i).get();
        return this;
    }

    public MotorConfig d(double d) {
        m_D = new TunableNumber(m_loggingName + "/PIDF/D", d).get();
        return this;
    }

    public MotorConfig ff(double ff) {
        m_FF = new TunableNumber(m_loggingName + "/PIDF/FF", ff).get();
        return this;
    }

    public MotorConfig minPower(double minPower) {
        m_minPower = new TunableNumber(m_loggingName + "/PowerRange/minPower", minPower).get();
        return this;
    }

    public MotorConfig maxPower(double maxPower) {
        m_maxPower = new TunableNumber(m_loggingName + "/PowerRange/maxPower", maxPower).get();
        return this;
    }

    public MotorConfig encoderOdometryFrequency(double encoderOdometryFrequency) {
        m_encoderOdometryFrequency = new TunableNumber(m_loggingName + "/EncoderOdometryFrequency",
                encoderOdometryFrequency).get();
        return this;
    }

    // Not Tunable
    // TODO change with Tunable Boolean if you want to.
    public MotorConfig isInverted(boolean isInverted) {
        m_isInverted = isInverted;
        return this;
    }

    public String name() {
        return m_loggingName;
    }

    public int motorCan() {
        return m_motorCan;
    }

    public double positionTolerance() {
        return m_positionTolerance;
    }

    public double speedTolerance() {
        return m_speedTolerance;
    }

    public double p() {
        return m_P;
    }

    public double i() {
        return m_I;
    }

    public double d() {
        return m_D;
    }

    public double ff() {
        return m_FF;
    }

    public double minPower() {
        return m_minPower;
    }

    public double maxPower() {
        return m_maxPower;
    }

    public double encoderOdometryFrequency() {
        return m_encoderOdometryFrequency;
    }

    public boolean isInverted() {
        return m_isInverted;
    }
}
