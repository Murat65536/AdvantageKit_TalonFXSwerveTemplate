package frc.robot.auton;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc.robot.FieldConstants;
import frc.robot.generated.choreo.ChoreoTraj;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.stream.Stream;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;

/**
 * Every Choreo path must pass under a trench with room to spare. maple-sim does not model the
 * trench legs, so a path that clips one still scores in sim; only this geometry check catches it.
 */
class TrenchClearanceTest {
  /**
   * Required side gap between the bumpers and the trench opening, on the planned path. The most any
   * path can have is (1.279 - 0.886) / 2 = 0.1965 m, with the robot centred and square to the
   * trench. The trajectory controller strayed up to 0.094 m from the path while under the trench
   * ({@code ChoreoTrackingHarness} on Left_Trench), so the plan must leave more than that. 0.15 m
   * clears it by 5.6 cm and leaves 4.7 cm of the maximum for Choreo's lane tolerance.
   */
  static final double MIN_PLANNED_CLEARANCE_METERS = 0.15;

  static Stream<ChoreoTraj> trajectories() {
    return ChoreoTraj.ALL_TRAJECTORIES.values().stream().filter(t -> t.segment().isEmpty());
  }

  @ParameterizedTest
  @MethodSource("trajectories")
  void plannedPathClearsTrenches(ChoreoTraj choreoTraj) {
    Trajectory<SwerveSample> trajectory =
        Choreo.<SwerveSample>loadTrajectory(choreoTraj.name()).orElseThrow();
    double halfSide = DriveConstants.BUMPER_WIDTH_METERS / 2.0;
    double worst = Double.POSITIVE_INFINITY;
    double worstTime = 0.0;
    for (SwerveSample sample : trajectory.samples()) {
      double clearance = FieldConstants.trenchClearance(sample.getPose(), halfSide);
      if (clearance < worst) {
        worst = clearance;
        worstTime = sample.t;
      }
    }
    double recorded = worst;
    double recordedTime = worstTime;
    assertTrue(
        recorded >= MIN_PLANNED_CLEARANCE_METERS,
        () ->
            String.format(
                "%s: %.3f m trench clearance at t=%.2f s; need %.3f m",
                choreoTraj.name(), recorded, recordedTime, MIN_PLANNED_CLEARANCE_METERS));
  }

  @ParameterizedTest
  @MethodSource("trajectories")
  void generatedStartPoseMatchesTheTrajectory(ChoreoTraj choreoTraj) {
    // StartPoseCheck and the chooser use ChoreoTraj.initialPoseBlue(); a stale codegen would warn
    // the drive team about the wrong spot.
    Trajectory<SwerveSample> trajectory =
        Choreo.<SwerveSample>loadTrajectory(choreoTraj.name()).orElseThrow();
    var start = trajectory.getInitialPose(false).orElseThrow();
    assertEquals(start.getX(), choreoTraj.initialPoseBlue().getX(), 1e-4);
    assertEquals(start.getY(), choreoTraj.initialPoseBlue().getY(), 1e-4);
  }
}
