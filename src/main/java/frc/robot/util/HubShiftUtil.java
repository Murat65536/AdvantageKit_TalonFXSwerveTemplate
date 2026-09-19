package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

/**
 * Tracks which alliance hub is active during teleop in 2026 REBUILT.
 *
 * <p>The FMS game-specific message is a single character ('R' or 'B') naming the alliance whose hub
 * goes INACTIVE first (the alliance that scored more fuel in auto). That alliance's hub is active
 * in shifts 2 and 4. Shift boundaries by match time remaining: transition (&gt;130 s), shift 1
 * (130-105), shift 2 (105-80), shift 3 (80-55), shift 4 (55-30), endgame (&lt;30, always active).
 *
 * <p>The match time reported to the robot is approximate, so never use this to block shooting; use
 * it for driver feedback only.
 */
public final class HubShiftUtil {
  private HubShiftUtil() {}

  public static final double TELEOP_DURATION = 140.0;
  public static final double TRANSITION_END = 130.0;
  public static final double SHIFT_LENGTH = 25.0;
  public static final double ENDGAME_START = 30.0;

  public enum Shift {
    TRANSITION,
    SHIFT_1,
    SHIFT_2,
    SHIFT_3,
    SHIFT_4,
    ENDGAME
  }

  /**
   * Returns the alliance whose hub goes inactive first, from game data. Empty if no game data has
   * been received yet.
   */
  public static Optional<Alliance> getInactiveFirstAlliance() {
    String gameData = DriverStation.getGameSpecificMessage();
    if (gameData.isEmpty()) {
      return Optional.empty();
    }
    return switch (gameData.charAt(0)) {
      case 'R' -> Optional.of(Alliance.Red);
      case 'B' -> Optional.of(Alliance.Blue);
      default -> Optional.empty();
    };
  }

  /** Shift for a given teleop match time remaining (seconds). */
  public static Shift getShift(double matchTime) {
    if (matchTime > TRANSITION_END) {
      return Shift.TRANSITION;
    } else if (matchTime > TRANSITION_END - SHIFT_LENGTH) {
      return Shift.SHIFT_1;
    } else if (matchTime > TRANSITION_END - 2 * SHIFT_LENGTH) {
      return Shift.SHIFT_2;
    } else if (matchTime > TRANSITION_END - 3 * SHIFT_LENGTH) {
      return Shift.SHIFT_3;
    } else if (matchTime > ENDGAME_START) {
      return Shift.SHIFT_4;
    }
    return Shift.ENDGAME;
  }

  /**
   * Whether our hub is active at the given teleop match time remaining. Returns true when the game
   * data is unavailable (the hub is active during the transition, and we should not stop the driver
   * from shooting if we simply don't know).
   */
  public static boolean isHubActive(double matchTime) {
    Optional<Alliance> inactiveFirst = getInactiveFirstAlliance();
    if (inactiveFirst.isEmpty()) {
      return true;
    }
    boolean ourHubActiveInShift1 =
        inactiveFirst.get() != DriverStation.getAlliance().orElse(Alliance.Blue);
    return switch (getShift(matchTime)) {
      case TRANSITION, ENDGAME -> true;
      case SHIFT_1, SHIFT_3 -> ourHubActiveInShift1;
      case SHIFT_2, SHIFT_4 -> !ourHubActiveInShift1;
    };
  }

  /** Whether our hub is active right now (teleop only; always true otherwise). */
  public static boolean isHubActive() {
    if (!DriverStation.isTeleopEnabled()) {
      return true;
    }
    return isHubActive(DriverStation.getMatchTime());
  }

  /**
   * Seconds until our hub next becomes active, or 0 if it is active now. Uses the approximate match
   * time.
   */
  public static double secondsUntilHubActive(double matchTime) {
    if (isHubActive(matchTime)) {
      return 0.0;
    }
    // Inactive shifts end at the next shift boundary.
    double nextBoundary =
        switch (getShift(matchTime)) {
          case SHIFT_1 -> TRANSITION_END - SHIFT_LENGTH;
          case SHIFT_2 -> TRANSITION_END - 2 * SHIFT_LENGTH;
          case SHIFT_3 -> TRANSITION_END - 3 * SHIFT_LENGTH;
          case SHIFT_4 -> ENDGAME_START;
          default -> matchTime;
        };
    return Math.max(0.0, matchTime - nextBoundary);
  }
}
