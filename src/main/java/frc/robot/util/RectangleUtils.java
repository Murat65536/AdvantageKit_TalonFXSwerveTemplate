package frc.robot.util;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

/** Utility methods for checking robot interaction with field rectangles. */
public final class RectangleUtils {
  private RectangleUtils() {}

  /**
   * Determines whether the robot is driving into any rectangle in the provided set.
   *
   * <p>The robot is considered to be "driving through" a rectangle when its footprint (a circle of
   * {@code robotRadius}) touches the rectangle and its translational velocity points toward the
   * rectangle at or above {@code speedThreshold}.
   */
  public static boolean drivingThroughRect(
      List<Rectangle2d> rects,
      Translation2d robotPos,
      double vx,
      double vy,
      double robotRadius,
      double speedThreshold) {
    for (Rectangle2d rect : rects) {
      if (drivingThroughRect(rect, robotPos, vx, vy, robotRadius, speedThreshold)) {
        return true;
      }
    }
    return false;
  }

  /** Determines whether the robot's footprint overlaps any rectangle in the provided set. */
  public static boolean inRect(
      List<Rectangle2d> rects, Translation2d robotPos, double robotRadius) {
    for (Rectangle2d rect : rects) {
      if (rect.nearest(robotPos).getDistance(robotPos) <= robotRadius) {
        return true;
      }
    }
    return false;
  }

  private static boolean drivingThroughRect(
      Rectangle2d rect,
      Translation2d robotPos,
      double vx,
      double vy,
      double robotRadius,
      double speedThreshold) {
    if (Math.hypot(vx, vy) < speedThreshold) {
      return false;
    }
    Translation2d nearest = rect.nearest(robotPos);
    if (nearest.getDistance(robotPos) > robotRadius) {
      return false;
    }
    // Positive dot product between velocity and the displacement toward the rectangle means the
    // robot is moving into it.
    double dx = nearest.getX() - robotPos.getX();
    double dy = nearest.getY() - robotPos.getY();
    return vx * dx + vy * dy > 0.0;
  }
}
