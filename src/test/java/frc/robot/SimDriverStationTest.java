package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.junit.jupiter.api.Test;

/**
 * The sim HAL starts with alliance station 0 (Unknown). Autos are built with alliance flipping on,
 * and ChoreoLib kills any routine started while the alliance is unknown ("Alliance not known when
 * starting routine" / "Killed an auto loop"), so selecting an auto in sim did nothing.
 */
class SimDriverStationTest {
  @Test
  void simStartsWithAKnownAllianceSoAutosRun() {
    assertTrue(HAL.initialize(500, 0));

    Robot.configureSimDriverStation();
    DriverStation.refreshData();

    assertEquals(Alliance.Blue, DriverStation.getAlliance().orElse(null));
    assertTrue(DriverStation.isDSAttached());
  }
}
