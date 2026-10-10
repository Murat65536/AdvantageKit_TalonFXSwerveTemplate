package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Game Manual TU22 Table 6-3 and section 6.5. We are Blue and game data is 'B' (Blue scored more in
 * auto), so Blue's hub is inactive in shift 1 (2:10-1:45 = 130-105 s) and shift 3 (80-55 s), and
 * active in shift 2 (105-80 s). Fuel still counts for 3 s after the hub deactivates.
 */
class HubShiftShotTest {
  @BeforeAll
  static void blueWithGameDataB() {
    HAL.initialize(500, 0);
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.setGameSpecificMessage("B");
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
  }

  @Test
  void shotLandingMidInactiveShiftDoesNotCount() {
    assertFalse(HubShiftUtil.wouldShotCount(120.0, 1.0));
  }

  @Test
  void shotLandingInsideTheThreeSecondGraceCounts() {
    // Hub deactivates at 80 s; released at 79, lands at 78: 2 s after deactivation.
    assertTrue(HubShiftUtil.wouldShotCount(79.0, 1.0));
  }

  @Test
  void shotLandingWellAfterTheGraceDoesNotCount() {
    // Lands at 74 s: 6 s after deactivation, past the grace plus the match-time margin.
    assertFalse(HubShiftUtil.wouldShotCount(75.0, 1.0));
  }

  @Test
  void shotReleasedBeforeActivationThatLandsAfterItCounts() {
    // Hub activates at 105 s; released at 106, lands at 104.5.
    assertTrue(HubShiftUtil.wouldShotCount(106.0, 1.5));
  }

  @Test
  void shotLandingClearlyBeforeActivationDoesNotCount() {
    assertFalse(HubShiftUtil.wouldShotCount(110.0, 1.0));
  }

  @Test
  void activeShiftCounts() {
    assertTrue(HubShiftUtil.wouldShotCount(95.0, 1.0));
  }

  @Test
  void noMatchTimeMeansNoFieldSoNeverBlock() {
    // getMatchTime() is -1 when practising without a match timer.
    assertTrue(HubShiftUtil.wouldShotCount(-1.0, 1.0));
  }
}
