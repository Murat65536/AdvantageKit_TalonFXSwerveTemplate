package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.auton.Autos;
import org.junit.jupiter.api.Test;

/**
 * Right Trench is Left Trench mirrored across the field's long axis, for when a partner takes the
 * left trench. Separate class because sim tests cannot share a JVM (build.gradle forks per class).
 */
class AutoScoringRightTrenchHarness {
  /** Same floor as Left Trench: the first volley alone empties the full 40-fuel hopper. */
  private static final int MIN_RIGHT_TRENCH_FUEL = 40;

  @Test
  void rightTrenchFollowsItsPathAndScores() {
    AutoScoringHarness.Result result =
        AutoScoringHarness.run("RightTrench", Alliance.Blue, Autos::rightTrench, "Right_Trench");
    assertTrue(
        result.maxPathDeviationMeters() <= AutoScoringHarness.MAX_PATH_DEVIATION_METERS,
        String.format("Robot strayed %.2f m from the path", result.maxPathDeviationMeters()));
    assertTrue(
        result.scored() >= MIN_RIGHT_TRENCH_FUEL,
        "Right Trench scored "
            + result.scored()
            + " fuel; expected at least "
            + MIN_RIGHT_TRENCH_FUEL);
  }
}
