package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.auton.Autos;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

/**
 * Center Depot: carry the preload to the depot, sweep it, and shoot everything from beside the hub.
 * Separate class because sim tests cannot share a JVM (build.gradle forks per class).
 */
class AutoScoringDepotHarness {
  /**
   * A down-the-wall sweep that started at the far end of the depot scored 40 in each of 3 runs,
   * with all 24 depot fuel picked up (the sim intake over-counts, so the real total is 32). A path
   * that got stuck against the depot fuel scored 10, which this bar catches.
   */
  private static final int MIN_CENTER_DEPOT_FUEL = 24;

  // Off because the path starts its sweep at the tower end of the depot. In maple-sim the gap
  // between the tower upright (a solid 47 in wall at x = 1.07 m, up to y = 4.64 m) and the depot's
  // first row of fuel (y = 5.46 m) is 0.81 m, narrower than the 0.886 m robot. As drawn, the robot
  // clips the upright, loses its odometry and misses the depot (3 of 8 scored); entering higher
  // jams it against the fuel instead (5 of 6 runs). The real tower may leave more room than maple's
  // model, so the path is kept as the team drew it until it has been run on the field.
  @Disabled("Center_Depot clips maple-sim's tower model; see comment")
  @Test
  void centerDepotCollectsFromTheDepotAndScores() {
    AutoScoringHarness.Result result =
        AutoScoringHarness.run(
            "CenterDepot", Alliance.Blue, Autos::centerDepot, "Center_Depot", true);
    assertTrue(
        result.scored() >= MIN_CENTER_DEPOT_FUEL,
        "Center Depot auto scored "
            + result.scored()
            + " fuel; expected at least "
            + MIN_CENTER_DEPOT_FUEL);
    assertTrue(
        result.maxPathDeviationMeters() <= AutoScoringHarness.MAX_PATH_DEVIATION_METERS,
        String.format("Robot strayed %.2f m from the path", result.maxPathDeviationMeters()));
  }
}
