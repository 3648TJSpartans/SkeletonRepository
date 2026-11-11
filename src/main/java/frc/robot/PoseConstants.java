package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.miscTunables.TunableRotation3d;
import frc.robot.util.miscTunables.TunableTranslation3d;
import frc.robot.util.zoneCalc.Polygon;
import frc.robot.util.zoneCalc.Rectangle;

public class PoseConstants {

  public static final Pose2d testPose = new Pose2d(0, 0, Rotation2d.fromDegrees(90));
  public static final double fieldLength = 16.540988;
  public static final double fieldWidth = 8.042;

  public static final Rectangle test =
      new Rectangle("testZone", new Translation2d(0, 0), new Translation2d(1, 1));
  public static final Polygon testPolygon =
      AllianceFlipUtil.apply(
          new Polygon(
              "testPolygon",
              new Translation2d(0, 0),
              new Translation2d(2, 4),
              new Translation2d(4, 0)));

  public static final Rectangle field =
      new Rectangle("field", new Translation2d(0, 0), new Translation2d(fieldLength, fieldWidth));
}
