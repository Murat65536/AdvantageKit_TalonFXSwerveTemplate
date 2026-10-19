package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.SystemsCheck.Status;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.extension.Extension;
import frc.robot.subsystems.extension.ExtensionIO;
import frc.robot.subsystems.extension.ExtensionIOSim;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.rollers.RollerIO;
import frc.robot.subsystems.rollers.RollerIOSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * The pit systems check must pass a healthy robot and catch the faults a match causes: an unplugged
 * CAN cable, a jammed roller, a limit switch that never trips, and a mechanism that got slower than
 * it was when the baseline was saved.
 */
class SystemsCheckTest {
  private SystemsCheck.MemoryStore baseline;

  @BeforeEach
  void enabledOnTheCart() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setFmsAttached(false);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
    baseline = new SystemsCheck.MemoryStore();
  }

  @AfterEach
  void reset() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
    SimHooks.resumeTiming();
  }

  /** Steps the scheduler and sim clock in 20 ms loops until the command ends (or 30 s). */
  private static void runToCompletion(Command command) {
    CommandScheduler.getInstance().schedule(command);
    for (int i = 0; i < 1500 && command.isScheduled(); i++) {
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(0.02);
    }
    assertTrue(!command.isScheduled(), "check did not finish within 30 s");
  }

  private static Feeder simFeeder() {
    return new Feeder(new RollerIOSim(FeederConstants.CONFIG));
  }

  @Test
  void healthyRollerPassesAndRecordsItsSpinUpTime() {
    SystemsCheck check = new SystemsCheck(baseline);
    check.addRoller("Feeder", simFeeder(), FeederConstants.FEED_VELOCITY);
    runToCompletion(check.run());
    var result = check.results().get("Feeder");
    assertEquals(Status.PASS, result.status(), result.detail());
    assertTrue(result.metric() > 0.0 && result.metric() < 1.0, "spin-up " + result.metric());
    assertEquals(Status.PASS, check.overall());
  }

  @Test
  void disconnectedRollerFailsWithoutBeingRun() {
    boolean[] commanded = {false};
    RollerIO unplugged =
        new RollerIO() {
          @Override
          public void setVelocity(AngularVelocity velocity) {
            commanded[0] = true;
          }
        };
    SystemsCheck check = new SystemsCheck(baseline);
    check.addRoller("Feeder", new Feeder(unplugged), FeederConstants.FEED_VELOCITY);
    runToCompletion(check.run());
    assertEquals(Status.FAIL, check.results().get("Feeder").status());
    assertTrue(check.results().get("Feeder").detail().contains("disconnected"));
    assertTrue(!commanded[0], "a disconnected motor must not be driven");
    assertEquals(Status.FAIL, check.overall());
  }

  @Test
  void jammedRollerFailsAtTheCap() {
    RollerIO jammed =
        new RollerIO() {
          @Override
          public void updateInputs(RollerIOInputs inputs) {
            inputs.connected = true;
            inputs.velocity = RPM.zero();
          }
        };
    SystemsCheck check = new SystemsCheck(baseline);
    check.addRoller("Feeder", new Feeder(jammed), FeederConstants.FEED_VELOCITY);
    runToCompletion(check.run());
    var result = check.results().get("Feeder");
    assertEquals(Status.FAIL, result.status());
    assertTrue(result.detail().contains("0 RPM"), result.detail());
  }

  @Test
  void slowerThanBaselineWarns() {
    SystemsCheck healthy = new SystemsCheck(baseline);
    healthy.addRoller("Feeder", simFeeder(), FeederConstants.FEED_VELOCITY);
    runToCompletion(healthy.run());
    double healthyTime = healthy.results().get("Feeder").metric();
    // Pretend the healthy robot was far quicker than this one is now.
    baseline.put("Feeder", healthyTime / (SystemsCheck.DRIFT_RATIO * 1.5));

    CommandScheduler.getInstance().unregisterAllSubsystems();
    SystemsCheck later = new SystemsCheck(baseline);
    later.addRoller("Feeder", simFeeder(), FeederConstants.FEED_VELOCITY);
    runToCompletion(later.run());
    assertEquals(Status.WARN, later.results().get("Feeder").status());
    assertEquals(Status.WARN, later.overall());
  }

  @Test
  void savedBaselineIsTheLastRunsMetrics() {
    SystemsCheck check = new SystemsCheck(baseline);
    check.addRoller("Feeder", simFeeder(), FeederConstants.FEED_VELOCITY);
    runToCompletion(check.run());
    runToCompletion(check.saveBaseline());
    assertEquals(check.results().get("Feeder").metric(), baseline.get("Feeder").getAsDouble());
  }

  @Test
  void baselineIsNotSavedFromARunWithAFailure() {
    RollerIO unplugged = new RollerIO() {};
    SystemsCheck check = new SystemsCheck(baseline);
    check.addRoller("Feeder", new Feeder(unplugged), FeederConstants.FEED_VELOCITY);
    runToCompletion(check.run());
    runToCompletion(check.saveBaseline());
    assertTrue(baseline.get("Feeder").isEmpty());
  }

  @Test
  void intakeRollerPassesWhenItSpinsForward() {
    // The intake is open-loop voltage; this stand-in reports the same RPM-per-volt as IntakeIOSim
    // without needing a maple-sim drivetrain.
    IntakeIO roller =
        new IntakeIO() {
          private double volts;

          @Override
          public void setRollerVoltage(Voltage voltage) {
            volts = voltage.in(Volts);
          }

          @Override
          public void updateInputs(IntakeIOInputs inputs) {
            inputs.rollerConnected = true;
            inputs.rollerVelocity = RPM.of(IntakeConstants.ROLLER_RPM_PER_VOLT * volts);
          }
        };
    SystemsCheck check = new SystemsCheck(baseline);
    check.addIntake(new Intake(roller));
    runToCompletion(check.run());
    var result = check.results().get("Intake");
    assertEquals(Status.PASS, result.status(), result.detail());
  }

  @Test
  void hoodReachesBothEndsOfItsAllowedRange() {
    SystemsCheck check = new SystemsCheck(baseline);
    check.addHood(new Hood(new HoodIOSim()));
    runToCompletion(check.run());
    // 55, the soft minimum: the 50 deg stop must never be commanded.
    for (String name : new String[] {"Hood to 55 deg", "Hood to 76 deg"}) {
      assertEquals(Status.PASS, check.results().get(name).status(), name);
    }
  }

  @Test
  void extensionGoesOutAndTheLimitSwitchStopsItComingBack() {
    SystemsCheck check = new SystemsCheck(baseline);
    check.addExtension(new Extension(new ExtensionIOSim()));
    runToCompletion(check.run());
    assertEquals(Status.PASS, check.results().get("Extension out").status());
    assertEquals(Status.PASS, check.results().get("Extension in (limit switch)").status());
  }

  @Test
  void limitSwitchStuckPressedFailsTheExtension() {
    ExtensionIO stuck =
        new ExtensionIO() {
          @Override
          public void updateInputs(ExtensionIOInputs inputs) {
            inputs.connected = true;
            inputs.reverseLimitPressed = true;
          }
        };
    SystemsCheck check = new SystemsCheck(baseline);
    check.addExtension(new Extension(stuck));
    runToCompletion(check.run());
    assertEquals(Status.FAIL, check.results().get("Extension out").status());
  }

  @Test
  void passiveCheckReportsAMissingDevice() {
    SystemsCheck check = new SystemsCheck(baseline);
    check.addPresence("Gyro", () -> false);
    runToCompletion(check.run());
    assertEquals(Status.FAIL, check.results().get("Gyro").status());
  }

  @Test
  void refusesToRunOnTheField() {
    DriverStationSim.setFmsAttached(true);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
    boolean[] ran = {false};
    SystemsCheck check = new SystemsCheck(baseline);
    check.addPresence("Gyro", () -> ran[0] = true);
    runToCompletion(check.run());
    assertTrue(!ran[0], "must never move mechanisms with the FMS attached");
    assertEquals(Status.SKIPPED, check.overall());
  }

  @Test
  void disablingMidCheckMarksTheRestSkipped() {
    SystemsCheck check = new SystemsCheck(baseline);
    check.addRoller("Feeder", simFeeder(), FeederConstants.FEED_VELOCITY);
    Command run = check.run();
    CommandScheduler.getInstance().schedule(run);
    CommandScheduler.getInstance().run();
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
    for (int i = 0; i < 10; i++) {
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(0.02);
    }
    assertEquals(Status.SKIPPED, check.results().get("Feeder").status());
  }

  /** Module stand-in: steer slews at 720 deg/s, drive speed settles with a 0.1 s time constant. */
  private static final class FakeModule implements ModuleIO {
    boolean turnEncoderConnected = true;
    double turnDeg;
    double turnTargetDeg;
    double driveRadPerSec;
    double driveTargetRadPerSec;

    @Override
    public void setTurnPosition(Rotation2d rotation) {
      turnTargetDeg = rotation.getDegrees();
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
      driveTargetRadPerSec = velocityRadPerSec;
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
      double step = 720.0 * 0.02;
      if (turnEncoderConnected) {
        turnDeg += Math.max(-step, Math.min(step, turnTargetDeg - turnDeg));
      }
      driveRadPerSec += (driveTargetRadPerSec - driveRadPerSec) * (0.02 / 0.1);
      inputs.driveConnected = true;
      inputs.turnConnected = true;
      inputs.turnEncoderConnected = turnEncoderConnected;
      inputs.turnPosition = Rotation2d.fromDegrees(turnDeg);
      inputs.turnAbsolutePosition = inputs.turnPosition;
      inputs.driveVelocityRadPerSec = driveRadPerSec;
    }
  }

  private static Drive fakeDrive(FakeModule... modules) {
    return new Drive(new GyroIO() {}, modules[0], modules[1], modules[2], modules[3]);
  }

  /** Swerve steps run in the robot loop order: subsystem periodic, then commands. */
  private static void runWithDrive(Command command) {
    runToCompletion(command);
  }

  @Test
  void healthySwervePassesEveryModule() {
    Drive drive = fakeDrive(new FakeModule(), new FakeModule(), new FakeModule(), new FakeModule());
    SystemsCheck check = new SystemsCheck(baseline);
    check.addSwerve(drive);
    runWithDrive(check.run());
    for (String module : new String[] {"FL", "FR", "BL", "BR"}) {
      for (String part : new String[] {" steer", " drive forward", " drive reverse"}) {
        var result = check.results().get("Swerve " + module + part);
        assertEquals(Status.PASS, result.status(), module + part + ": " + result.detail());
      }
    }
  }

  @Test
  void deadSteerEncoderFailsOnlyThatModule() {
    FakeModule broken = new FakeModule();
    broken.turnEncoderConnected = false;
    Drive drive = fakeDrive(new FakeModule(), broken, new FakeModule(), new FakeModule());
    SystemsCheck check = new SystemsCheck(baseline);
    check.addSwerve(drive);
    runWithDrive(check.run());
    assertEquals(Status.FAIL, check.results().get("Swerve FR steer").status());
    assertEquals(Status.PASS, check.results().get("Swerve FL steer").status());
    assertEquals(Status.FAIL, check.overall());
  }
}
