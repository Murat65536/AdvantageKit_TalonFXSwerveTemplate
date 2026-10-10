package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.DriverRumble;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Two features rumble the driver controller: the one-button shot (while aiming) and the hub
 * activation warning. They used to write the controller directly, so whichever wrote last won, and
 * the warning ending sent "rumble 0" and cut the aiming rumble while the driver was still shooting.
 * {@link DriverRumble} owns the controller and sends the stronger of the two.
 */
class DriverRumbleTest {
  // Blue goes inactive first, so blue's hub is inactive in shift 1 (130-105 s remaining) and
  // becomes active at 105 s.
  private static final double WARNING_TIME = 107.0; // 2 s before activation
  private static final double ACTIVE_TIME = 104.0; // shift 2, our hub active

  private final double[] controller = {0.0};
  private final DriverRumble rumble = new DriverRumble(level -> controller[0] = level);

  @BeforeAll
  static void initHal() {
    HAL.initialize(500, 0);
  }

  @BeforeEach
  void teleopEnabledBlue() {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.setGameSpecificMessage("B");
    setEnabled(true);
    setMatchTime(WARNING_TIME);
  }

  @AfterEach
  void reset() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
  }

  private static void setMatchTime(double seconds) {
    DriverStationSim.setMatchTime(seconds);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
  }

  private static void setEnabled(boolean enabled) {
    DriverStationSim.setEnabled(enabled);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
  }

  private static void tick() {
    CommandScheduler.getInstance().run();
  }

  @Test
  void aimingRumbleSurvivesHubActivation() {
    CommandScheduler.getInstance().schedule(rumble.hubWarning(), rumble.whileAiming());
    tick();

    setMatchTime(ACTIVE_TIME); // warning over; driver still holding the aim trigger
    tick();

    assertEquals(
        DriverRumble.AIMING_LEVEL,
        controller[0],
        1e-9,
        "driver is still holding the aim trigger, so the aiming rumble must still be on");
  }

  @Test
  void strongerSignalWinsWhileBothAreActive() {
    setMatchTime(105.1); // 0.1 s before activation: warning ramp ~0.97, above aiming's 0.8
    CommandScheduler.getInstance().schedule(rumble.hubWarning(), rumble.whileAiming());
    tick();

    double warning = DriverRumble.hubWarningLevel(105.1);
    assertTrue(warning > DriverRumble.AIMING_LEVEL, "precondition: warning is the stronger one");
    assertEquals(warning, controller[0], 1e-9);
  }

  @Test
  void hubWarningAloneRampsThenStops() {
    CommandScheduler.getInstance().schedule(rumble.hubWarning());
    tick();
    assertEquals(1.0 - 2.0 / DriverRumble.HUB_WARNING_SECONDS, controller[0], 1e-6);

    setMatchTime(ACTIVE_TIME);
    tick();
    assertEquals(
        0.0, controller[0], 1e-9, "warning must release the rumble once the hub is active");
  }

  @Test
  void releasingAimTriggerStopsAimingRumble() {
    setMatchTime(ACTIVE_TIME); // no warning in play
    Command aiming = rumble.whileAiming();
    CommandScheduler.getInstance().schedule(aiming);
    tick();
    assertEquals(DriverRumble.AIMING_LEVEL, controller[0], 1e-9);

    aiming.cancel();
    assertEquals(0.0, controller[0], 1e-9);
  }

  @Test
  void hubWarningLevelRampsOnlyInFinalSeconds() {
    // Our hub inactive, 20 s to go: silent.
    assertEquals(0.0, DriverRumble.hubWarningLevel(125.0), 1e-9);
    // Exactly at the start of the window: silent, then ramping.
    assertEquals(0.0, DriverRumble.hubWarningLevel(105.0 + DriverRumble.HUB_WARNING_SECONDS), 1e-9);
    assertEquals(
        0.5, DriverRumble.hubWarningLevel(105.0 + DriverRumble.HUB_WARNING_SECONDS / 2), 1e-9);
    // Our hub active: silent regardless.
    assertEquals(0.0, DriverRumble.hubWarningLevel(ACTIVE_TIME), 1e-9);
  }

  @Test
  void teleopTriggerRunsWarningAndStopsItOnDisable() {
    // Same binding and same order as the robot: RobotContainer binds in robotInit while DISABLED,
    // then teleop enables. Trigger captures the condition when the binding is made and only
    // schedules on a false->true edge, so binding while already enabled would never start it.
    setEnabled(false);
    Command warning = rumble.hubWarning();
    RobotModeTriggers.teleop().whileTrue(warning);
    tick();
    assertFalse(warning.isScheduled(), "warning must not run while disabled");

    setEnabled(true);
    tick(); // trigger sees the rising edge and schedules the warning
    tick(); // warning executes
    assertTrue(warning.isScheduled(), "warning should start when teleop is enabled");
    assertTrue(controller[0] > 0.0, "warning should be rumbling 2 s before activation");

    setEnabled(false);
    tick();
    assertFalse(warning.isScheduled(), "warning should stop when teleop ends");
    assertEquals(0.0, controller[0], 1e-9, "rumble must not be left on after teleop");
  }
}
