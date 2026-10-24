package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.commands.ManualShot.*;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.hood.HoodConstants;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** Manual mode's values, steps and limits, which come from the 2026RobotProject. */
class ManualShotTest {
  private final boolean[] enabled = {false};
  private final ManualShot manual = new ManualShot(() -> enabled[0]);

  @BeforeAll
  static void initHal() {
    HAL.initialize(500, 0);
  }

  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** The adjust commands are one-shot and run while disabled, so one scheduler run applies them. */
  private void press(Command command, int times) {
    for (int i = 0; i < times; i++) {
      CommandScheduler.getInstance().schedule(command);
      CommandScheduler.getInstance().run();
    }
  }

  @Test
  void startsAtTheOldProjectsDefaults() {
    assertEquals(2900.0, manual.getRpm());
    assertEquals(70.0, manual.getHoodDeg());
    assertEquals(6750.0, MAX_RPM, "Vortex free speed 6784 rounded down to a 50 RPM step");
  }

  @Test
  void rpmStepsBy50AndStaysInRange() {
    press(manual.adjustRpm(RPM_STEP), 2);
    assertEquals(3000.0, manual.getRpm());
    press(manual.adjustRpm(-RPM_STEP), 100);
    assertEquals(MIN_RPM, manual.getRpm());
    press(manual.adjustRpm(RPM_STEP), 200);
    assertEquals(MAX_RPM, manual.getRpm());
  }

  @Test
  void hoodStepsBy2AndNeverGoesPastTheSoftMinimum() {
    press(manual.adjustHood(HOOD_STEP_DEG), 1);
    assertEquals(72.0, manual.getHoodDeg());
    press(manual.adjustHood(HOOD_STEP_DEG), 10);
    assertEquals(HoodConstants.MAX_HOOD_ANGLE.in(Degrees), manual.getHoodDeg());
    press(manual.adjustHood(-HOOD_STEP_DEG), 30);
    assertEquals(
        HoodConstants.HOOD_SOFT_MIN_ANGLE.in(Degrees),
        manual.getHoodDeg(),
        "manual mode must not drive the hood into the 50 deg stop that breaks it");
  }

  @Test
  void resetRestoresBothDefaults() {
    press(manual.adjustRpm(RPM_STEP), 5);
    press(manual.adjustHood(-HOOD_STEP_DEG), 3);
    press(manual.reset(), 1);
    assertEquals(DEFAULT_RPM, manual.getRpm());
    assertEquals(DEFAULT_HOOD_DEG, manual.getHoodDeg());
  }

  @Test
  void followsTheToggle() {
    assertFalse(manual.isEnabled());
    enabled[0] = true;
    assertTrue(manual.isEnabled());
  }
}
