package frc.robot;

import static frc.robot.subsystems.shooter.ShooterConstants.FEED_STOP_DEBOUNCE;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ShootCommands;
import frc.robot.subsystems.extension.Extension;
import frc.robot.subsystems.extension.ExtensionIO;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.rollers.RollerIO;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * The feed path must not move fuel unless the shot is ready. Time is paused and stepped by hand so
 * the debounce window is exercised exactly rather than depending on how fast the test machine is.
 */
class AutoFeedGateTest {
  private static final double DT = 0.02;
  private static final double DEBOUNCE = FEED_STOP_DEBOUNCE.in(Units.Seconds);

  private final boolean[] ready = {false};
  private Feeder feeder;
  private Command autoFeed;

  @BeforeAll
  static void initHal() {
    HAL.initialize(500, 0);
  }

  @BeforeEach
  void setUp() {
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    SimHooks.pauseTiming();
    feeder = new Feeder(new RollerIO() {});
    autoFeed =
        ShootCommands.autoFeed(
            () -> ready[0],
            feeder,
            new Indexer(new RollerIO() {}),
            new Intake(new IntakeIO() {}),
            new Extension(new ExtensionIO() {}));
  }

  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
    SimHooks.resumeTiming();
    DriverStationSim.resetData();
  }

  private void tick() {
    SimHooks.stepTiming(DT);
    CommandScheduler.getInstance().run();
  }

  private void runFor(double seconds, String whenFeedingFails) {
    for (int i = 0; i < Math.round(seconds / DT); i++) {
      tick();
      assertFalse(feeder.isRunningForward(), whenFeedingFails + " (t=" + (i + 1) * DT + " s)");
    }
  }

  @Test
  void doesNotFeedAtStartWhenNotReady() {
    CommandScheduler.getInstance().schedule(autoFeed);
    runFor(1.0, "fed with ready=false right after the command started");
  }

  @Test
  void doesNotFeedWhenRestartedRightAfterReleasingMidShot() {
    CommandScheduler.getInstance().schedule(autoFeed);
    ready[0] = true;
    tick();
    tick();
    assertTrue(feeder.isRunningForward(), "precondition: feeding while ready");

    // Driver lets go mid-shot, the robot loses aim, and the driver presses again straight away.
    autoFeed.cancel();
    ready[0] = false;
    tick();
    CommandScheduler.getInstance().schedule(autoFeed);
    runFor(DEBOUNCE * 2, "re-pressing inside the debounce window fed blind");
  }

  @Test
  void keepsFeedingThroughABriefDropoutThenStops() {
    CommandScheduler.getInstance().schedule(autoFeed);
    ready[0] = true;
    tick();
    tick();
    assertTrue(feeder.isRunningForward(), "precondition: feeding while ready");

    // The flywheel dips on each shot; a dip shorter than the debounce must not interrupt feeding.
    ready[0] = false;
    for (int i = 0; i < Math.round(DEBOUNCE / DT) - 2; i++) {
      tick();
      assertTrue(feeder.isRunningForward(), "a dropout shorter than the debounce stopped feeding");
    }

    // A dropout longer than the debounce must stop it.
    for (int i = 0; i < 4; i++) {
      tick();
    }
    assertFalse(feeder.isRunningForward(), "feeding kept going after the debounce expired");
  }
}
