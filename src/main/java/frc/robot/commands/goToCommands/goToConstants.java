package frc.robot.commands.goToCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.TunableNumber;

public class goToConstants {

        public static final TunableNumber drivekP = new TunableNumber("goTo/driveKP", 2);
        public static final TunableNumber drivekD = new TunableNumber("goTo/driveKD", 0);
        public static final TunableNumber thetakP = new TunableNumber("goTo/thetakP", 5);
        public static final TunableNumber thetakD = new TunableNumber("goTo/thetaKD", 0);
        public static final TunableNumber driveMaxVelocity =
                        new TunableNumber("goTo/driveMaxVelocity", .5);
        public static final TunableNumber driveMaxAcceleration =
                        new TunableNumber("goTo/driveMaxAcceleration", 1.5);
        public static final double thetaMaxVelocity = Units.degreesToRadians(360.0);
        public static final double thetaMaxAcceleration = Units.degreesToRadians(720.0);
        public static final TunableNumber driveTolerance = new TunableNumber("goTo/driveKp", 0.05);
        public static final TunableNumber thetaTolerance =
                        new TunableNumber("goTo/driveKp", Units.degreesToRadians(1));
        public static final double ffMinRadius = 0.2;
        public static final double ffMaxRadius = 0.6;

        public static class PoseConstants {
                public final static Pose2d examplePose =
                                new Pose2d(3.57, 2.75, Rotation2d.fromDegrees(60));
                public final static Pose2d examplePose2 =
                                new Pose2d(1, 1, Rotation2d.fromDegrees(90));
                public final static double fieldLength = 17.548;
                public final static double fieldWidth = 8.042;

                private static Pose2d[] examplePoseListBlue =
                                new Pose2d[] {new Pose2d(2, 2, Rotation2d.fromDegrees(180)),
                                                new Pose2d(3, 3, Rotation2d.fromDegrees(270))};

                public final static Pose2d[] examplePoseList =
                                AllianceFlipUtil.apply(examplePoseListBlue);

                public static enum AutonState {
                        EXAMPLE1, EXAMPLE2, EXAMPLE3, DEFAULT
                }

        }
}
