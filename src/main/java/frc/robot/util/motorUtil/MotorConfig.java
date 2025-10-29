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

    public MotorConfig(String name) {
        m_loggingName = "MotorIOs/" + name;
    }

    public void name(String name) {
        m_loggingName = "MotorIOs/" + name;
    }

    public void motorCan(int motorCAN) {
        m_motorCan = (int) new TunableNumber(m_loggingName + "/motorCAN", motorCAN).get();
    }

    public void positionTolerance(double positionTolerance) {
        m_positionTolerance = positionTolerance;
    }

    public void speedTolerance(double speedTolerance) {
        m_speedTolerance = speedTolerance;
    }

    public void p(double p) {
        m_P = new TunableNumber(m_loggingName + "/PIDF/P", p).get();
    }

    public void i(double i) {
        m_I = new TunableNumber(m_loggingName + "/PIDF/I", i).get();
    }

    public void d(double d) {
        m_D = new TunableNumber(m_loggingName + "/PIDF/D", d).get();
    }

    public void ff(double ff) {
        m_FF = new TunableNumber(m_loggingName + "/PIDF/FF", ff).get();
    }

    public void minPower(double minPower) {
        m_minPower = new TunableNumber(m_loggingName + "/PowerRange/minPower", minPower).get();
    }

    public void maxPower(double maxPower) {
        m_maxPower = new TunableNumber(m_loggingName + "/PowerRange/maxPower", maxPower).get();
    }

    public void encoderOdometryFrequency(double encoderOdometryFrequency) {
        m_encoderOdometryFrequency = new TunableNumber(m_loggingName + "/EncoderOdometryFrequency",
                encoderOdometryFrequency).get();
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
}
