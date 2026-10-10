package frc.robot;

import static frc.robot.FieldConstants.*;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class FieldConstantsTest {
  /**
   * maple-sim's {@code RebuiltHub.blueHubPose} X, the point the simulator scores against. Our own
   * {@link FieldConstants#BLUE_HUB_TRANSLATION} is computed from {@code
   * HUB_EDGE_DISTANCE_FROM_DRIVER_STATION + HUB_LENGTH / 2}, which currently lands 1.10 in further
   * downfield. See {@link #hubCenterAgreesWithTheSimulatorItIsScoredAgainst()}.
   */
  private static final double MAPLE_HUB_X = 4.5974;

  /**
   * The robot aims at {@link FieldConstants#BLUE_HUB_TRANSLATION} but maple-sim decides whether a
   * shot counts using its own hub pose.
   *
   * <p>Verified against the official 2026 REBUILT Game Manual (TU22, Section 5.3, Section 5.4,
   * Figure 5-6, Figure 5-7): our 158.6 in edge distance and 4.62534 m center is the true official
   * field geometry. Maple-sim 0.4.0-beta's {@code RebuiltHub.blueHubPose = 4.5974 m} implies an
   * edge distance of 157.5 in (1.10 in further upfield), an upstream simulator discrepancy.
   *
   * <p>Because the hub scoring box is 47.0 in wide, a 1.10 in difference leaves 98% overlap (1.166
   * m of 1.194 m). This test pins the discrepancy at < 1.2 in so it cannot grow silently.
   */
  @Test
  void hubCenterAgreesWithTheSimulatorItIsScoredAgainst() {
    double ours = BLUE_HUB_TRANSLATION.getX();
    double delta = Math.abs(ours - MAPLE_HUB_X);
    assertTrue(
        delta < Units.inchesToMeters(1.2),
        String.format(
            "hub center X drifted from maple-sim's scoring pose: ours=%.5f m maple=%.5f m "
                + "(%.3f in apart). Reconcile HUB_EDGE_DISTANCE_FROM_DRIVER_STATION (%.1f in) "
                + "against the game manual -- maple-sim implies %.4f in.",
            ours,
            MAPLE_HUB_X,
            delta / Units.inchesToMeters(1.0),
            Units.metersToInches(HUB_EDGE_DISTANCE_FROM_DRIVER_STATION),
            Units.metersToInches(MAPLE_HUB_X - HUB_LENGTH / 2.0)));
  }

  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
  }

  @AfterEach
  void resetDriverStation() {
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
  }

  private static void setAlliance(AllianceStationID station) {
    DriverStationSim.setAllianceStationId(station);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
  }

  @Test
  void hubTranslationTracksAlliance() {
    setAlliance(AllianceStationID.Blue1);
    assertEquals(BLUE_HUB_TRANSLATION, getHubTranslation());

    setAlliance(AllianceStationID.Red1);
    assertEquals(FlippingUtil.flipFieldPosition(BLUE_HUB_TRANSLATION), getHubTranslation());
  }

  @Test
  void targetIsHubWhenOnOwnSide() {
    setAlliance(AllianceStationID.Blue1);
    Translation2d robot = new Translation2d(2.0, FIELD_WIDTH / 2.0);
    assertFalse(isPastHub(robot));
    assertEquals(BLUE_HUB_TRANSLATION, targetPosition(robot));
  }

  @Test
  void targetIsNearestOwnCornerWhenPastHub() {
    setAlliance(AllianceStationID.Blue1);
    Translation2d robotLeft = new Translation2d(HUB_FAR_EDGE_X + 1.0, FIELD_WIDTH - 1.0);
    assertTrue(isPastHub(robotLeft));
    assertEquals(
        new Translation2d(CORNER_TARGET_X_OFFSET, FIELD_WIDTH - CORNER_TARGET_Y_OFFSET),
        targetPosition(robotLeft));

    Translation2d robotRight = new Translation2d(HUB_FAR_EDGE_X + 1.0, 1.0);
    assertEquals(
        new Translation2d(CORNER_TARGET_X_OFFSET, CORNER_TARGET_Y_OFFSET),
        targetPosition(robotRight));
  }

  @Test
  void redTargetIsMirrored() {
    setAlliance(AllianceStationID.Red1);
    Translation2d robot = new Translation2d(flipX(HUB_FAR_EDGE_X + 1.0), 1.0);
    assertTrue(isPastHub(robot));
    assertEquals(
        new Translation2d(FIELD_LENGTH - CORNER_TARGET_X_OFFSET, CORNER_TARGET_Y_OFFSET),
        targetPosition(robot));
  }

  @Test
  void trenchIsFortySevenInchesDeepAndCentredOnTheHub() {
    // Game Manual TU22 section 5.6: 47.0 in deep. The trench AprilTags (IDs 22/23, welded layout)
    // sit at x = 4.6631 and 4.5882 m: centre 4.6257 m, the hub centre to 0.4 mm.
    var blueLeft = TRENCH_RECTANGLES.get(0);
    assertEquals(Units.inchesToMeters(47.0), blueLeft.getXWidth(), 1e-9);
    assertEquals(BLUE_HUB_TRANSLATION.getX(), blueLeft.getCenter().getX(), 1e-9);
    assertEquals(4.6257, blueLeft.getCenter().getX(), 0.001);
  }

  @Test
  void leftTrenchLaneCentreMatchesTheTrenchAprilTags() {
    // Tags 22/23 are "approximately centered on the opening under the TRENCH arm" (section 5.6).
    assertEquals(7.4248, LEFT_TRENCH_LANE_CENTER_Y, 0.006);
    assertEquals(FIELD_WIDTH - LEFT_TRENCH_LANE_CENTER_Y, RIGHT_TRENCH_LANE_CENTER_Y, 1e-9);
  }

  /** Robot footprint from DriveConstants.mapleSimConfig: 0.886 m square bumpers. */
  private static final double HALF_SIDE = 0.886 / 2.0;

  @Test
  void centredSquareRobotHasEqualClearanceBothSides() {
    Pose2d pose =
        new Pose2d(BLUE_HUB_TRANSLATION.getX(), LEFT_TRENCH_LANE_CENTER_Y, Rotation2d.kZero);
    assertEquals((TRENCH_OPENING_WIDTH - 0.886) / 2.0, trenchClearance(pose, HALF_SIDE), 1e-9);
    Pose2d sideways = new Pose2d(pose.getTranslation(), Rotation2d.kCW_90deg);
    assertEquals((TRENCH_OPENING_WIDTH - 0.886) / 2.0, trenchClearance(sideways, HALF_SIDE), 1e-9);
  }

  @Test
  void diagonalRobotSticksOutFurther() {
    Pose2d pose =
        new Pose2d(
            BLUE_HUB_TRANSLATION.getX(), LEFT_TRENCH_LANE_CENTER_Y, Rotation2d.fromDegrees(45));
    double halfDiagonal = HALF_SIDE * Math.sqrt(2);
    assertEquals(TRENCH_OPENING_WIDTH / 2.0 - halfDiagonal, trenchClearance(pose, HALF_SIDE), 1e-9);
  }

  @Test
  void offCentreRobotHitsTheLeg() {
    Pose2d pose =
        new Pose2d(BLUE_HUB_TRANSLATION.getX(), LEFT_TRENCH_LANE_CENTER_Y - 0.25, Rotation2d.kZero);
    assertTrue(trenchClearance(pose, HALF_SIDE) < 0);
  }

  @Test
  void robotBesideTheTrenchIsUnconstrained() {
    Pose2d beforeTrench = new Pose2d(3.0, LEFT_TRENCH_LANE_CENTER_Y, Rotation2d.kZero);
    assertEquals(Double.POSITIVE_INFINITY, trenchClearance(beforeTrench, HALF_SIDE));
    Pose2d besideHub = new Pose2d(BLUE_HUB_TRANSLATION.getX(), FIELD_WIDTH / 2.0, Rotation2d.kZero);
    assertEquals(Double.POSITIVE_INFINITY, trenchClearance(besideHub, HALF_SIDE));
  }

  @Test
  void bumperCornerJustReachingUnderTheTrenchCounts() {
    // Centre 0.2 m before the trench's near face: the front bumper is already 0.243 m under it.
    double nearFace = BLUE_HUB_TRANSLATION.getX() - Units.inchesToMeters(47.0) / 2.0;
    Pose2d pose = new Pose2d(nearFace - 0.2, RIGHT_TRENCH_LANE_CENTER_Y + 0.25, Rotation2d.kZero);
    assertTrue(trenchClearance(pose, HALF_SIDE) < 0);
  }

  @Test
  void redTrenchesAreMirrored() {
    Pose2d pose =
        new Pose2d(
            FIELD_LENGTH - BLUE_HUB_TRANSLATION.getX(),
            RIGHT_TRENCH_LANE_CENTER_Y,
            Rotation2d.kZero);
    assertEquals((TRENCH_OPENING_WIDTH - 0.886) / 2.0, trenchClearance(pose, HALF_SIDE), 1e-9);
  }

  @Test
  void trenchLaneDetection() {
    assertTrue(isInTrenchLane(new Translation2d(5.0, 0.5)));
    assertTrue(isInTrenchLane(new Translation2d(5.0, FIELD_WIDTH - 0.5)));
    assertFalse(isInTrenchLane(new Translation2d(5.0, FIELD_WIDTH / 2.0)));
  }
}
