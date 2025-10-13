package frc.robot.subsystems.absoluteEncoder;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
    private final AbsEncoderIO io;

    public Turret(AbsEncoderIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        // io.updateValues();
        Logger.recordOutput("Turret/position", io.getPosition());
    }

    public double getPosition() {
        return io.getPosition();
    }

    public void setTo(double setpoint) {
        Logger.recordOutput("Turret/setPos", setpoint);
        io.setTo(setpoint);
    }

    public void stop() {
        io.stop();
    }

    public void setSpeed(double speed) {
        io.setSpeed(speed);
    }
}
