package frc.robot.util.motorUtil;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableNumber;

public abstract class MotorIO extends SubsystemBase {
    private final String name;
    private final TunableNumber m_poseTolerance;
    private final TunableNumber m_speedTolerance;
    private double m_speedSetpoint;
    private double m_positionSetpoint;
    private double m_powerSetpoint;

    public MotorIO(String loggingName, double poseTolerance, double speedTolerance) {
        this.name = loggingName;
        m_poseTolerance =
                new TunableNumber(loggingName + "/Tolerances/poseTolerance", poseTolerance);
        m_speedTolerance =
                new TunableNumber(loggingName + "/Tolerances/speedTolerance", speedTolerance);
    }

    public abstract double getPosition();

    public abstract double getSpeed();

    public void setPosition(double position) {
        m_positionSetpoint = position;
    }

    public void setSpeed(double speed) {
        m_speedSetpoint = speed;
    }

    public final boolean positionInTolerance() {
        return Math.abs(getPosition() - m_positionSetpoint) < m_poseTolerance.get();
    }

    public final boolean speedInTolerance() {
        return Math.abs(getSpeed() - m_speedSetpoint) < m_speedTolerance.get();
    }

    public abstract void setPower(double power){
        m_powerSetpoint = power;
    };

    public final void updateValues() {
        Logger.recordOutput(name + "/pose", getPosition());
        Logger.recordOutput(name + "/speed", getSpeed());
        Logger.recordOutput(name + "/setPose", m_positionSetpoint);
        Logger.recordOutput(name + "/setSpeed", m_speedSetpoint);
        Logger.recordOutput(name + "/poseInTolerance", positionInTolerance());
        Logger.recordOutput(name + "/speedInTolerance", speedInTolerance());
        logger.recordOutput(name + "/setPower", m_powerSetpoint)
    }

    public abstract void setEncoder(double setpoint);

    public final void resetEncoder() {
        setEncoder(0.0);
    }

    public final void stop() {
        setSpeed(0.0);
    }

    public final String getName() {
        return name;
    }

    @Override
    public void periodic() {
        updateValues();
    }
}
