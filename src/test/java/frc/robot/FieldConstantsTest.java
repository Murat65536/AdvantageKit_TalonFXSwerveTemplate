package frc.robot;

import static frc.robot.FieldConstants.*;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class FieldConstantsTest {
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
