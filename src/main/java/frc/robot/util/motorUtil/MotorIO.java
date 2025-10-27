package frc.robot.util.motorUtil;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableNumber;

public abstract class MotorIO {
    private final String name;
    private final TunableNumber m_poseTolerance;
    private final TunableNumber m_speedTolerance;
    private double m_speedSetpoint;
    private double m_positionSetpoint;

    public MotorIO(String loggingName, double poseTolerance, double speedTolerance) {
        this.name = "MotorIOs/" + loggingName;
        m_poseTolerance =
                new TunableNumber(loggingName + "/Tolerances/poseTolerance", poseTolerance);
        m_speedTolerance =
                new TunableNumber(loggingName + "/Tolerances/speedTolerance", speedTolerance);
    }

    public double getPosition() {
        return 0.0;
    }

    public double getSpeed() {
        return 0.0;
    }

    public void setPosition(double position) {
        m_positionSetpoint = position;
    }

    public void setSpeed(double speed) {
        m_speedSetpoint = speed;
    }

    public boolean positionInTolerance() {
        return Math.abs(getPosition() - m_positionSetpoint) < m_poseTolerance.get();
    }

    public boolean speedInTolerance() {
        return Math.abs(getSpeed() - m_speedSetpoint) < m_speedTolerance.get();
    }

    public void setPower(double power) {}

    public void updateValues() {
        Logger.recordOutput("name" + "/pose", getPosition());
        Logger.recordOutput("name" + "/speed", getSpeed());
        Logger.recordOutput("name" + "/setPose", m_positionSetpoint);
        Logger.recordOutput("name" + "/setSpeed", m_speedSetpoint);
        Logger.recordOutput("name" + "/poseInTolerance", positionInTolerance());
        Logger.recordOutput("name" + "/speedInTolerance", speedInTolerance());
    }

    public void setEncoder(double setpoint) {

    }

    public void resetEncoder() {
        setEncoder(0.0);
    }

    public void stop() {
        setSpeed(0.0);
    }

    public String getName() {
        return name;
    }
}
