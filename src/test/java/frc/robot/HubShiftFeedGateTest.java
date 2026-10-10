package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.commands.ShootCommands;
import frc.robot.subsystems.shooter.ShooterMath.ShotSolution;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Feeding into our own inactive hub scores nothing (Game Manual TU22 section 6.4.1). Blue with game
 * data 'B' is inactive in shift 1 (130-105 s), so at 120 s left a 1 s shot is wasted.
 */
class HubShiftFeedGateTest {
  private static final ShotSolution ONE_SECOND_SHOT =
      new ShotSolution(
          Translation2d.kZero,
          MetersPerSecond.of(8),
          RPM.of(2500),
          Degrees.of(60),
          Seconds.of(1.0),
          3.0,
          true);

  @BeforeAll
  static void blueTeleopShiftOne() {
    HAL.initialize(500, 0);
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.setGameSpecificMessage("B");
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setMatchTime(120.0);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
  }

  @Test
  void blocksFeedIntoOurInactiveHub() {
    assertFalse(ShootCommands.hubAllowsFeed(ONE_SECOND_SHOT, false));
  }

  @Test
  void driverOverrideAlwaysFeeds() {
    assertTrue(ShootCommands.hubAllowsFeed(ONE_SECOND_SHOT, true));
  }

  @Test
  void noShotSolutionLeavesTheDecisionToTheOtherGates() {
    assertTrue(ShootCommands.hubAllowsFeed(null, false));
  }
}
