package frc.robot;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.List;

/** 2026 REBUILT field geometry. All dimensions are for the blue alliance unless flipped. */
public final class FieldConstants {
  private FieldConstants() {}

  public static final double FIELD_LENGTH = FlippingUtil.fieldSizeX;
  public static final double FIELD_WIDTH = FlippingUtil.fieldSizeY;

  // Hub
  public static final double HUB_EDGE_DISTANCE_FROM_DRIVER_STATION = Units.inchesToMeters(158.6);
  public static final double HUB_LENGTH = Units.inchesToMeters(47.0);
  public static final double HUB_FAR_EDGE_X = HUB_EDGE_DISTANCE_FROM_DRIVER_STATION + HUB_LENGTH;
  public static final Translation2d BLUE_HUB_TRANSLATION =
      new Translation2d(
          HUB_EDGE_DISTANCE_FROM_DRIVER_STATION + HUB_LENGTH / 2.0, FIELD_WIDTH / 2.0);
  public static final double HUB_HEIGHT_METERS = Units.inchesToMeters(104.0);

  // Bumps and trenches (the obstacles flanking the hub on both alliance sides)
  public static final double BUMP_WIDTH = Units.inchesToMeters(73.0);
  public static final double BUMP_DEPTH = Units.inchesToMeters(44.4);
  public static final double TRENCH_WIDTH = Units.inchesToMeters(65.65);

  private static final double LEFT_TRENCH_Y_OFFSET = flipY(TRENCH_WIDTH);
  private static final double RIGHT_TRENCH_Y_OFFSET = 0.0;
  private static final double LEFT_BUMP_Y_OFFSET = LEFT_TRENCH_Y_OFFSET - BUMP_WIDTH;
  private static final double RIGHT_BUMP_Y_OFFSET = TRENCH_WIDTH;
  private static final double RED_SIDE_X =
      flipX(HUB_EDGE_DISTANCE_FROM_DRIVER_STATION + BUMP_DEPTH);

  private static final Rectangle2d BUMP =
      new Rectangle2d(Translation2d.kZero, new Translation2d(BUMP_DEPTH, BUMP_WIDTH));
  private static final Rectangle2d TRENCH =
      new Rectangle2d(Translation2d.kZero, new Translation2d(BUMP_DEPTH, TRENCH_WIDTH));

  public static final List<Rectangle2d> BUMP_RECTANGLES =
      List.of(
          place(BUMP, HUB_EDGE_DISTANCE_FROM_DRIVER_STATION, LEFT_BUMP_Y_OFFSET),
          place(BUMP, HUB_EDGE_DISTANCE_FROM_DRIVER_STATION, RIGHT_BUMP_Y_OFFSET),
          place(BUMP, RED_SIDE_X, LEFT_BUMP_Y_OFFSET),
          place(BUMP, RED_SIDE_X, RIGHT_BUMP_Y_OFFSET));

  public static final List<Rectangle2d> TRENCH_RECTANGLES =
      List.of(
          place(TRENCH, HUB_EDGE_DISTANCE_FROM_DRIVER_STATION, LEFT_TRENCH_Y_OFFSET),
          place(TRENCH, HUB_EDGE_DISTANCE_FROM_DRIVER_STATION, RIGHT_TRENCH_Y_OFFSET),
          place(TRENCH, RED_SIDE_X, LEFT_TRENCH_Y_OFFSET),
          place(TRENCH, RED_SIDE_X, RIGHT_TRENCH_Y_OFFSET));

  // Corner pass targets used when the robot is on the far side of the hub and cannot score.
  public static final double CORNER_TARGET_X_OFFSET = 1.0;
  public static final double CORNER_TARGET_Y_OFFSET = 1.0;

  private static Rectangle2d place(Rectangle2d rect, double x, double y) {
    return rect.transformBy(new Transform2d(new Translation2d(x, y), Rotation2d.kZero));
  }

  public static Alliance getAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue);
  }

  /** Returns the hub position in the current alliance's field coordinate frame. */
  public static Translation2d getHubTranslation() {
    return getAlliance() == Alliance.Red
        ? FlippingUtil.flipFieldPosition(BLUE_HUB_TRANSLATION)
        : BLUE_HUB_TRANSLATION;
  }

  /** True when the robot is past the far edge of its alliance hub (on the opponent's half). */
  public static boolean isPastHub(Translation2d robotPosition) {
    return getAlliance() == Alliance.Blue
        ? robotPosition.getX() > HUB_FAR_EDGE_X
        : robotPosition.getX() < flipX(HUB_FAR_EDGE_X);
  }

  /**
   * Returns the point the shooter should aim at: the alliance hub, or, when the robot is past the
   * hub and cannot score, the nearest corner on the alliance's own side of the field so fuel is
   * passed back toward the hub.
   */
  public static Translation2d targetPosition(Translation2d robotPosition) {
    if (!isPastHub(robotPosition)) {
      return getHubTranslation();
    }
    double x =
        getAlliance() == Alliance.Blue
            ? CORNER_TARGET_X_OFFSET
            : FIELD_LENGTH - CORNER_TARGET_X_OFFSET;
    Translation2d leftCorner = new Translation2d(x, FIELD_WIDTH - CORNER_TARGET_Y_OFFSET);
    Translation2d rightCorner = new Translation2d(x, CORNER_TARGET_Y_OFFSET);
    return robotPosition.getDistance(leftCorner) <= robotPosition.getDistance(rightCorner)
        ? leftCorner
        : rightCorner;
  }

  /** True when the given field position is inside either trench lane (near a long field wall). */
  public static boolean isInTrenchLane(Translation2d position) {
    return position.getY() < TRENCH_WIDTH || position.getY() > FIELD_WIDTH - TRENCH_WIDTH;
  }

  public static double flipX(double x) {
    return FIELD_LENGTH - x;
  }

  public static double flipY(double y) {
    return FIELD_WIDTH - y;
  }
}
