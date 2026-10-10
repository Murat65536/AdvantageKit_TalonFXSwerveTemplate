package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.auton.Autos;
import org.junit.jupiter.api.Test;

/**
 * Runs Left Trench as Red. The path is authored for Blue and flipped at runtime by ChoreoLib, while
 * the aim target is flipped by {@link FieldConstants}; if the two flips disagree, Red drives the
 * wrong path or shoots at the wrong hub, which the Blue run can never catch. Separate class because
 * sim tests cannot share a JVM (build.gradle forks per class).
 *
 * <p>Fuel counts are not compared with Blue's. Both alliances collect the same 32 fuel on the first
 * sweep, but maple-sim 0.4.0-beta re-launches scored fuel from the wrong spot for Red:
 * RebuiltHub.redShootPoses mirrors the Blue outlets with FieldMirroringUtils.FIELD_WIDTH = 17.548
 * m, while the field is 16.541 m long, so Red's outlets sit 1.007 m toward the Red wall, inside the
 * hub instead of on its neutral-zone face. Blue's second sweep picks up that recycled fuel (about
 * 12); Red's picks up none, so Red scores about 40 against Blue's 50-68. What must match is the
 * path, and that fuel lands in the Red hub at all ({@link AutoScoringHarness} counts only our own
 * hub).
 */
class AutoScoringRedHarness {
  /** First volley alone empties the full 40-fuel hopper; below that, the first cycle broke. */
  private static final int MIN_LEFT_TRENCH_FUEL = 40;

  @Test
  void leftTrenchFollowsTheFlippedPathAndScoresAsRed() {
    AutoScoringHarness.Result result =
        AutoScoringHarness.run("LeftTrench-Red", Alliance.Red, Autos::leftTrench, "Left_Trench");
    assertTrue(
        result.maxPathDeviationMeters() <= AutoScoringHarness.MAX_PATH_DEVIATION_METERS,
        String.format(
            "Red robot strayed %.2f m from the flipped path", result.maxPathDeviationMeters()));
    assertTrue(
        result.scored() >= MIN_LEFT_TRENCH_FUEL,
        "Left Trench scored "
            + result.scored()
            + " fuel in the Red hub; expected at least "
            + MIN_LEFT_TRENCH_FUEL);
  }
}
