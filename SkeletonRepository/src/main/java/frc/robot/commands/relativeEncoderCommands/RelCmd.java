package frc.robot.commands.relativeEncoderCommands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.relativeEncoder.RelEncoder;

public class RelCmd extends Command {
    private final RelEncoder m_relEncoder;
    private final double angle;

    public RelCmd(RelEncoder relEncoder, double angle) {
        m_relEncoder = relEncoder;
        this.angle = angle;
        addRequirements(m_relEncoder);
    }

    @Override
    public void initialize() {
        Logger.recordOutput("relEncoder/CommandScheduled", true);
    }

    @Override
    public void execute() {
        Logger.recordOutput("relEncoder/setAngle", angle);
        Logger.recordOutput("relEncoder/angle", m_relEncoder.getPos());
        m_relEncoder.setTo(angle);
    }

    @Override
    public void end(boolean interrupted) {
        m_relEncoder.stop();
        Logger.recordOutput("relEncoder/CommandScheduled", false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}