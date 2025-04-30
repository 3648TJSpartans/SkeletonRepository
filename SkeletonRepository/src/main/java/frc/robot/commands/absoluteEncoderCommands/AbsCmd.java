package frc.robot.commands.absoluteEncoderCommands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.absoluteEncoder.AbsEncoder;

public class AbsCmd extends Command {
    private final AbsEncoder m_absEncoder;
    private final double angle;

    public AbsCmd(AbsEncoder absEncoder, double angle) {
        m_absEncoder = absEncoder;
        this.angle = angle;
        addRequirements(m_absEncoder);
    }

    @Override
    public void initialize() {
        Logger.recordOutput("absEncoder/CommandScheduled", true);
    }

    @Override
    public void execute() {
        Logger.recordOutput("absEncoder/setAngle", angle);
        Logger.recordOutput("absEncoder/angle", m_absEncoder.getPosition());
        m_absEncoder.setTo(angle);
    }

    @Override
    public void end(boolean interrupted) {
        m_absEncoder.stop();
        Logger.recordOutput("absEncoder/CommandScheduled", false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}