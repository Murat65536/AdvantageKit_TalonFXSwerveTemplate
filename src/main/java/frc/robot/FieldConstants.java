package frc.robot;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
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
  /**
   * Distance from the inside face of the alliance wall to the near (rear) face of the hub.
   *
   * <p>Source: Official 2026 REBUILT Game Manual (TU22), Section 5.3 (Alliance Zone depth 158.6
   * in), Section 5.4, and Figure 5-6 ("HUB distance to the ALLIANCE WALL"), which dimensions the
   * alliance wall to the back face of the hub as exactly 158.6 in.
   */
  public static final double HUB_EDGE_DISTANCE_FROM_DRIVER_STATION = Units.inchesToMeters(158.6);

  /** Width and depth of the square hub structure (Game Manual Section 5.4, Figure 5-7). */
  public static final double HUB_LENGTH = Units.inchesToMeters(47.0);

  public static final double HUB_FAR_EDGE_X = HUB_EDGE_DISTANCE_FROM_DRIVER_STATION + HUB_LENGTH;
  public static final Translation2d BLUE_HUB_TRANSLATION =
      new Translation2d(
          HUB_EDGE_DISTANCE_FROM_DRIVER_STATION + HUB_LENGTH / 2.0, FIELD_WIDTH / 2.0);
  public static final double HUB_HEIGHT_METERS = Units.inchesToMeters(104.0);

  /**
   * Scoring volume of the hub, taken from maple-sim (the authority that decides whether a shot
   * counts). Source: {@code RebuiltHub.blueHubPose = (4.5974, 4.034536, 1.5748)} passed to {@code
   * Goal(xDimension=47in, yDimension=47in, height=10in)}, and {@code Goal.checkValidity} -> {@code
   * positionChecker = box(xyBox, minZ = position.z, maxZ = position.z + height)}.
   *
   * <p>Fuel scores when its center is inside the 47in x 47in footprint AND its height is within
   * this 10in band. Note {@link #HUB_HEIGHT_METERS} (104in) is the physical tower height and is NOT
   * the scoring height -- aiming at it overshoots the window by ~0.8 m.
   */
  public static final double HUB_SCORING_Z_MIN = 1.5748;

  public static final double HUB_SCORING_Z_MAX = HUB_SCORING_Z_MIN + Units.inchesToMeters(10.0);

  /** Center of the scoring height band -- the height a shot should be solved to arrive at. */
  public static final double HUB_SCORING_Z_CENTER = (HUB_SCORING_Z_MIN + HUB_SCORING_Z_MAX) / 2.0;

  // Bumps and trenches (the obstacles flanking the hub on both alliance sides)
  public static final double BUMP_WIDTH = Units.inchesToMeters(73.0);
  public static final double BUMP_DEPTH = Units.inchesToMeters(44.4);
  /**
   * Trench structure, Game Manual TU22 section 5.6: 65.65 in wide (guardrail to bump), 47.0 in
   * deep, with a 50.34 in wide opening under the arm. The trench AprilTags (IDs 22/23 on the blue
   * side, welded layout x = 4.6631 / 4.5882 m) put its centre at x = 4.6257 m, the hub centre.
   */
  public static final double TRENCH_WIDTH = Units.inchesToMeters(65.65);

  public static final double TRENCH_DEPTH = Units.inchesToMeters(47.0);
  public static final double TRENCH_OPENING_WIDTH = Units.inchesToMeters(50.34);

  /**
   * Centre line of the opening under each trench arm; the opening starts at the guardrail (section
   * 5.6). Within 6 mm of the trench AprilTags (y = 7.4248 m), which the manual places
   * "approximately centered on the opening".
   */
  public static final double RIGHT_TRENCH_LANE_CENTER_Y = TRENCH_OPENING_WIDTH / 2.0;

  public static final double LEFT_TRENCH_LANE_CENTER_Y = FIELD_WIDTH - RIGHT_TRENCH_LANE_CENTER_Y;

  private static final double LEFT_TRENCH_Y_OFFSET = flipY(TRENCH_WIDTH);
  private static final double RIGHT_TRENCH_Y_OFFSET = 0.0;
  private static final double LEFT_BUMP_Y_OFFSET = LEFT_TRENCH_Y_OFFSET - BUMP_WIDTH;
  private static final double RIGHT_BUMP_Y_OFFSET = TRENCH_WIDTH;
  private static final double RED_SIDE_X =
      flipX(HUB_EDGE_DISTANCE_FROM_DRIVER_STATION + BUMP_DEPTH);

  private static final Rectangle2d BUMP =
      new Rectangle2d(Translation2d.kZero, new Translation2d(BUMP_DEPTH, BUMP_WIDTH));
  private static final Rectangle2d TRENCH =
      new Rectangle2d(Translation2d.kZero, new Translation2d(TRENCH_DEPTH, TRENCH_WIDTH));
  private static final double BLUE_TRENCH_X = BLUE_HUB_TRANSLATION.getX() - TRENCH_DEPTH / 2.0;
  private static final double RED_TRENCH_X = flipX(BLUE_TRENCH_X + TRENCH_DEPTH);

  public static final List<Rectangle2d> BUMP_RECTANGLES =
      List.of(
          place(BUMP, HUB_EDGE_DISTANCE_FROM_DRIVER_STATION, LEFT_BUMP_Y_OFFSET),
          place(BUMP, HUB_EDGE_DISTANCE_FROM_DRIVER_STATION, RIGHT_BUMP_Y_OFFSET),
          place(BUMP, RED_SIDE_X, LEFT_BUMP_Y_OFFSET),
          place(BUMP, RED_SIDE_X, RIGHT_BUMP_Y_OFFSET));

  public static final List<Rectangle2d> TRENCH_RECTANGLES =
      List.of(
          place(TRENCH, BLUE_TRENCH_X, LEFT_TRENCH_Y_OFFSET),
          place(TRENCH, BLUE_TRENCH_X, RIGHT_TRENCH_Y_OFFSET),
          place(TRENCH, RED_TRENCH_X, LEFT_TRENCH_Y_OFFSET),
          place(TRENCH, RED_TRENCH_X, RIGHT_TRENCH_Y_OFFSET));

  /**
   * Smallest side gap, in metres, between a square robot's bumpers and the edges of a trench
   * opening it is under; negative means the bumpers overlap the trench leg or guardrail. Positive
   * infinity when no part of the robot is under a trench.
   *
   * @param halfSide half the bumper-to-bumper width (the robot is treated as a square)
   */
  public static double trenchClearance(Pose2d pose, double halfSide) {
    double cos = Math.abs(pose.getRotation().getCos());
    double sin = Math.abs(pose.getRotation().getSin());
    // Half-extents of the rotated square's axis-aligned bounding box.
    double halfX = halfSide * (cos + sin);
    double halfY = halfSide * (sin + cos);
    double minX = pose.getX() - halfX;
    double maxX = pose.getX() + halfX;
    double minY = pose.getY() - halfY;
    double maxY = pose.getY() + halfY;
    double clearance = Double.POSITIVE_INFINITY;
    for (Rectangle2d trench : TRENCH_RECTANGLES) {
      Translation2d centre = trench.getCenter().getTranslation();
      boolean overlapsX =
          maxX > centre.getX() - trench.getXWidth() / 2.0
              && minX < centre.getX() + trench.getXWidth() / 2.0;
      boolean overlapsY =
          maxY > centre.getY() - trench.getYWidth() / 2.0
              && minY < centre.getY() + trench.getYWidth() / 2.0;
      if (!overlapsX || !overlapsY) {
        continue;
      }
      double laneCentre =
          centre.getY() > FIELD_WIDTH / 2.0
              ? LEFT_TRENCH_LANE_CENTER_Y
              : RIGHT_TRENCH_LANE_CENTER_Y;
      double low = laneCentre - TRENCH_OPENING_WIDTH / 2.0;
      double high = laneCentre + TRENCH_OPENING_WIDTH / 2.0;
      clearance = Math.min(clearance, Math.min(minY - low, high - maxY));
    }
    return clearance;
  }

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
