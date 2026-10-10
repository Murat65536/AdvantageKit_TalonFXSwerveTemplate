package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.Test;

/** With no game data (practice, or before teleop starts) the hub status is unknown: never block. */
class HubShiftNoDataTest {
  @Test
  void unknownHubStatusNeverBlocks() {
    HAL.initialize(500, 0);
    DriverStationSim.setGameSpecificMessage("");
    DriverStationSim.notifyNewData();
    assertTrue(HubShiftUtil.wouldShotCount(120.0, 1.0));
  }
}
