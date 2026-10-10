package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.auton.Autos;
import org.junit.jupiter.api.Test;

/**
 * The fallback auto for when a partner takes the trench: back away from the hub and shoot the
 * preload. Every preloaded fuel must score (Game Manual TU22 G303: at most 8 preloaded). Separate
 * class because sim tests cannot share a JVM (build.gradle forks per class).
 */
class AutoScoringPreloadHarness {
  @Test
  void centerPreloadScoresEveryPreloadedFuel() {
    AutoScoringHarness.Result result =
        AutoScoringHarness.run(
            "CenterPreload", Alliance.Blue, Autos::centerPreload, "Center_Preload");
    assertEquals(AutoScoringHarness.LEGAL_PRELOAD, result.scored());
    assertTrue(
        result.maxPathDeviationMeters() <= AutoScoringHarness.MAX_PATH_DEVIATION_METERS,
        String.format("Robot strayed %.2f m from the path", result.maxPathDeviationMeters()));
  }
}
