package frc.robot.auton;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.Optional;

/**
 * Compares where the robot is with where the selected auto starts, so a mis-placed robot shows up
 * on the dashboard before the match instead of as a crash into the trench.
 */
public final class StartPoseCheck {
  private StartPoseCheck() {}

  /**
   * Trench autos pass under a 1.279 m opening (Game Manual TU22 section 5.6) with 0.886 m bumpers,
   * leaving at most 19.65 cm a side. Left Trench is planned 18.9 cm clear, and the sim robot tracks
   * it 10.0 cm clear ({@code ChoreoTrackingHarness}, which asserts at least this tolerance). A
   * robot placed this far off starts this far off the path, so 5 cm keeps the tracked margin
   * positive.
   */
  public static final double TRANSLATION_TOLERANCE_METERS = 0.05;

  /**
   * At the bumper corner (0.626 m from centre: half the 0.886 m square's diagonal), 3 degrees moves
   * the corner 3.3 cm, the same order as {@link #TRANSLATION_TOLERANCE_METERS}.
   */
  public static final double HEADING_TOLERANCE_DEG = 3.0;

  public record Result(boolean ok, double translationErrorMeters, double headingErrorDeg) {}

  /**
   * @param autoStart the selected auto's start pose, already flipped for our alliance; empty when
   *     the auto has no fixed start (No Auto, characterization), which never warns.
   */
  public static Result check(Optional<Pose2d> autoStart, Pose2d robotPose) {
    if (autoStart.isEmpty()) {
      return new Result(true, 0.0, 0.0);
    }
    Pose2d start = autoStart.get();
    double translationError = start.getTranslation().getDistance(robotPose.getTranslation());
    double headingError = Math.abs(start.getRotation().minus(robotPose.getRotation()).getDegrees());
    boolean ok =
        translationError <= TRANSLATION_TOLERANCE_METERS && headingError <= HEADING_TOLERANCE_DEG;
    return new Result(ok, translationError, headingError);
  }
}
