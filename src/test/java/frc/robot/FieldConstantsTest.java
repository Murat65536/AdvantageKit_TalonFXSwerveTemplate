package frc.robot;

import static frc.robot.FieldConstants.*;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
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
   * shot counts using its own hub pose. If the two disagree, the shot table is validated at a
   * distance the robot never actually shoots from.
   *
   * <p>They currently disagree by 1.10 in: our 158.6 in edge distance implies a center of 4.62534
   * m, while maple-sim's 4.5974 m implies an edge distance of exactly 157.5 in. 157.5 being a round
   * number is suspicious -- one of the two is wrong and it needs a game-manual check, not a guess.
   * This test pins the current disagreement so it cannot grow silently, and will fail loudly if
   * anyone edits either number without reconciling them.
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
  void trenchLaneDetection() {
    assertTrue(isInTrenchLane(new Translation2d(5.0, 0.5)));
    assertTrue(isInTrenchLane(new Translation2d(5.0, FIELD_WIDTH - 0.5)));
    assertFalse(isInTrenchLane(new Translation2d(5.0, FIELD_WIDTH / 2.0)));
  }
}
