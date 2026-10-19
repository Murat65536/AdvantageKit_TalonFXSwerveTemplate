package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.hood.HoodConstants.*;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** The Pit-tab hood tools: short open-loop jogs and the zero button. */
class HoodPitCommandsTest {
  private static final double DT = 0.02;

  /** Records what the Hood asks of its IO; reports a fixed angle. */
  private static final class RecordingIO implements HoodIO {
    Double jogVolts = null;
    int zeroCalls = 0;
    Angle lastTarget = null;

    @Override
    public void updateInputs(HoodIOInputs inputs) {
      inputs.angle = Degrees.of(60.0);
    }

    @Override
    public void startJog(Voltage volts) {
      jogVolts = volts.in(Volts);
    }

    @Override
    public void stopJog() {
      jogVolts = null;
    }

    @Override
    public void zeroAtSteepStop() {
      zeroCalls++;
    }

    @Override
    public void setTargetAngle(Angle angle) {
      lastTarget = angle;
    }
  }

  private RecordingIO io;
  private Hood hood;

  @BeforeAll
  static void initHal() {
    HAL.initialize(500, 0);
  }

  @BeforeEach
  void setUp() {
    SimHooks.pauseTiming();
    io = new RecordingIO();
    hood = new Hood(io);
  }

  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
    SimHooks.resumeTiming();
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
  }

  private void setEnabled(boolean enabled, boolean fms) {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setFmsAttached(fms);
    DriverStationSim.setEnabled(enabled);
    DriverStationSim.notifyNewData();
  }

  private void tick() {
    SimHooks.stepTiming(DT);
    CommandScheduler.getInstance().run();
  }

  @Test
  void jogSteeperDrivesNegativeBrieflyThenHoldsWhereItStopped() {
    setEnabled(true, false);
    CommandScheduler.getInstance().schedule(hood.jogSteeper());
    tick();
    assertEquals(-HOOD_JOG_VOLTAGE.in(Volts), io.jogVolts, 1e-9);

    for (int i = 0; i < Math.round(HOOD_JOG_SECONDS / DT) + 2; i++) {
      tick();
    }
    assertEquals(null, io.jogVolts, "jog must stop on its own");
    assertEquals(60.0, io.lastTarget.in(Degrees), 1e-9, "hold where the jog left the hood");
  }

  @Test
  void jogShallowerDrivesPositive() {
    setEnabled(true, false);
    CommandScheduler.getInstance().schedule(hood.jogShallower());
    tick();
    assertEquals(HOOD_JOG_VOLTAGE.in(Volts), io.jogVolts, 1e-9);
  }

  @Test
  void jogRefusesOnTheField() {
    setEnabled(true, true);
    CommandScheduler.getInstance().schedule(hood.jogSteeper());
    tick();
    assertEquals(null, io.jogVolts);
  }

  @Test
  void disablingStopsAJog() {
    setEnabled(true, false);
    CommandScheduler.getInstance().schedule(hood.jogSteeper());
    tick();
    setEnabled(false, false);
    tick();
    assertEquals(null, io.jogVolts);
  }

  @Test
  void zeroWhileEnabledHoldsAtTheNewReading() {
    setEnabled(true, false);
    CommandScheduler.getInstance().schedule(hood.zeroAtSteepStop());
    tick();
    assertEquals(1, io.zeroCalls);
    // The old target would make the hood chase it after the reading jumps to MAX.
    assertEquals(MAX_HOOD_ANGLE.in(Degrees), io.lastTarget.in(Degrees), 1e-9);
  }

  @Test
  void zeroWorksWhileDisabled() {
    setEnabled(false, false);
    CommandScheduler.getInstance().schedule(hood.zeroAtSteepStop());
    tick();
    assertEquals(1, io.zeroCalls);
  }

  @Test
  void zeroRefusesOnTheField() {
    setEnabled(true, true);
    CommandScheduler.getInstance().schedule(hood.zeroAtSteepStop());
    tick();
    assertEquals(0, io.zeroCalls);
  }

  @Test
  void targetsNeverGoBelowTheSoftMinimum() {
    hood.setTargetAngle(MIN_HOOD_ANGLE);
    assertEquals(HOOD_SOFT_MIN_ANGLE.in(Degrees), io.lastTarget.in(Degrees), 1e-9);
    hood.setTargetAngleUntrimmed(Degrees.of(40.0));
    assertEquals(HOOD_SOFT_MIN_ANGLE.in(Degrees), io.lastTarget.in(Degrees), 1e-9);
  }

  @Test
  void trimCannotPushPastTheSoftMinimum() {
    setEnabled(false, false);
    for (int i = 0; i < 20; i++) {
      CommandScheduler.getInstance().schedule(hood.adjustTrim(Degrees.of(-0.5)));
      tick();
    }
    hood.setTargetAngle(HOOD_SOFT_MIN_ANGLE);
    assertEquals(HOOD_SOFT_MIN_ANGLE.in(Degrees), io.lastTarget.in(Degrees), 1e-9);
  }

  @Test
  void softMinimumLeavesMarginFromTheHardStop() {
    assertTrue(HOOD_SOFT_MIN_ANGLE.in(Degrees) > MIN_HOOD_ANGLE.in(Degrees));
    assertTrue(HOOD_SOFT_MIN_ANGLE.in(Degrees) < MAX_HOOD_ANGLE.in(Degrees));
  }

  @Test
  void zeroedJogStopsShallowerAtTheSoftMinimumOnly() {
    double soft = HOOD_SOFT_MIN_ANGLE.in(Degrees);
    assertTrue(jogAllowed(1.5, soft + 1.0, true), "shallower, above the limit");
    assertFalse(jogAllowed(1.5, soft, true), "shallower, at the limit");
    assertFalse(jogAllowed(1.5, soft - 2.0, true), "shallower, past the limit");
    assertTrue(jogAllowed(-1.5, soft - 2.0, true), "steeper always backs away from it");
    // Unzeroed, the angle means nothing; the person jogging decides.
    assertTrue(jogAllowed(1.5, soft - 2.0, false));
  }

  @Test
  void jogVoltageIsGentle() {
    assertTrue(HOOD_JOG_VOLTAGE.in(Volts) > 0.0);
    assertFalse(HOOD_JOG_VOLTAGE.in(Volts) > 3.0, "a jog is for creeping into a hard stop");
  }
}
