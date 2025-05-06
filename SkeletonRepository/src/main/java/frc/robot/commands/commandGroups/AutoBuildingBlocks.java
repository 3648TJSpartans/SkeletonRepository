package frc.robot.commands.commandGroups;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.goToCommands.DriveToPose;
import frc.robot.commands.relativeEncoderCommands.RelCmd;
import frc.robot.commands.absoluteEncoderCommands.AbsCmd;
import frc.robot.commands.goToCommands.DriveToNearest;
import frc.robot.subsystems.absoluteEncoder.AbsEncoder;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.relativeEncoder.RelEncoder;
import frc.robot.util.TunableNumber;

public class AutoBuildingBlocks {

    public static Command absCmd(AbsEncoder m_AbsEncoder, double setpoint) {
        return new AbsCmd(m_AbsEncoder, setpoint);
    }

    public static Command relCmd(RelEncoder m_RelEncoder, double setpoint) {
        return new RelCmd(m_RelEncoder, setpoint);
    }

    public static Command driveToPose(Drive m_drive, Pose2d point) {
        return new DriveToPose(m_drive, () -> point);
    }

    public static Command driveToNearest(Drive m_drive, Pose2d[] points) {
        return new DriveToNearest(m_drive, () -> points);
    }

}
