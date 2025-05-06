package frc.robot.subsystems.simpleMotor;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimpleMotor extends SubsystemBase {
    private final SimpleMotorIO io;

    public SimpleMotor(SimpleMotorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
    }

    public void stop() {
        io.stop();
    }

    public void setSpeed(double speed) {
        io.setSpeed(speed);
    }

    public double getSpeed(double speed) {
        return io.getSpeed();
    }
}
