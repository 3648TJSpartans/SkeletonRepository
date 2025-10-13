package frc.robot.commands.turretCommands;

import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;

public class TurretFollowCmd extends Command {
    private final Turret m_turret;
    private final Supplier<Translation2d> targetDisplacementSupplier;
    private final Supplier<Rotation2d> robotRotationSupplier;

    @FunctionalInterface
    public interface Rotation2dSupplier extends Supplier<Rotation2d> {
    }

    @FunctionalInterface
    public interface Translation2dSupplier extends Supplier<Translation2d> {
    }

    @FunctionalInterface
    public interface Pose2dSupplier extends Supplier<Pose2d> {
    }

    public TurretFollowCmd(Turret turret, Translation2dSupplier targetDisplacementSupplier,
            Rotation2d Supplier robotRotationSupplier) {
        this.m_turret = turret;
        this.targetDisplacementSupplier = targetDisplacementSupplier;
        this.robotRotationSupplier = robotRotationSupplier;
        addRequirements(m_turret);
    }

    public TurretFollowCmd(Turret turret, Pose2dSupplier robotPoseSupplier,
            Translation2dSupplier targetPoseSupplier) {
        this(turret, () -> );
    }


    @Override
    public void initialize() {
        Logger.recordOutput("Commands/TurretCommand/CommandScheduled", true);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        m_turret.stop();
        Logger.recordOutput("Commands/TurretCommand/CommandScheduled", false);
    }

    @Override
    public boolean isFinished() {
        /*
         * A finish state is essential to commands that are used in command groups (see command
         * group file for more details). The code below finishes the command if it's within a
         * certain constant margin, the setpointTolerance.
         */
        return false;
        // return ((position - TurretConstants.setpointTolerance) < m_turret.getPosition()
        // && m_turret.getPosition() < (position + TurretConstants.setpointTolerance));
    }

}
