package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Dashboard;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Module;
import frc.robot.subsystems.extension.Extension;
import frc.robot.subsystems.extension.ExtensionConstants;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerConstants;
import frc.robot.subsystems.rollers.RollerSubsystem;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterMath;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.Elastic;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.OptionalDouble;
import java.util.function.BiFunction;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Pit systems check, run between matches with the robot on its cart (wheels off the ground) and the
 * hopper empty. Each step drives one mechanism and times how long it takes to reach its target:
 *
 * <ul>
 *   <li>FAIL: the device is disconnected, or the mechanism never reaches its target.
 *   <li>WARN: it reached its target, but more than {@link #DRIFT_RATIO} times slower than the saved
 *       baseline. A bearing starting to bind, a belt slipping, or a motor losing a phase all show
 *       up here before they fail outright.
 *   <li>PASS: otherwise. With no baseline yet, reaching the target within the cap is a pass.
 * </ul>
 *
 * <p>The caps are deliberately generous; they only exist so a dead mechanism fails instead of
 * hanging the check. The real standard is the robot's own healthy baseline: run the check on a
 * known-good robot and press "Save Baseline". Metrics are stored with WPILib {@link Preferences},
 * which live on the roboRIO and survive redeploys.
 *
 * <p>Never runs with the FMS attached, and stops the moment the robot is disabled.
 */
public final class SystemsCheck {
  /** Slower than this multiple of the baseline time is a warning. */
  public static final double DRIFT_RATIO = 1.5;

  /** Upper bound on any single step: a mechanism that has not moved by now is broken. */
  static final double STEP_CAP_SECONDS = 3.0;

  /** Rollers and the shooter: fraction of the commanded speed that counts as "reached". */
  static final double SPEED_REACHED_FRACTION = 0.9;

  /**
   * The extension retract step allows this multiple of the timed extend, so a retract that needs
   * more travel than the extend covered still fits. Below the cap it is timed like everything else.
   */
  static final double RETRACT_TIME_MULTIPLE = 4.0;

  public enum Status {
    NOT_RUN,
    RUNNING,
    PASS,
    WARN,
    FAIL,
    SKIPPED
  }

  /** One row of the pit report. {@code metric} is the step's time in seconds; NaN if none. */
  public record Result(Status status, String detail, double metric) {}

  /** Where healthy-robot metrics are kept between runs. */
  public interface BaselineStore {
    OptionalDouble get(String step);

    void put(String step, double value);
  }

  /** Stores baselines in roboRIO {@link Preferences}, which persist across reboots and deploys. */
  public static final class PreferencesStore implements BaselineStore {
    private static final String PREFIX = "SystemsCheck/Baseline/";

    @Override
    public OptionalDouble get(String step) {
      return Preferences.containsKey(PREFIX + step)
          ? OptionalDouble.of(Preferences.getDouble(PREFIX + step, 0.0))
          : OptionalDouble.empty();
    }

    @Override
    public void put(String step, double value) {
      Preferences.setDouble(PREFIX + step, value);
    }
  }

  /** In-memory store for tests. */
  public static final class MemoryStore implements BaselineStore {
    private final Map<String, Double> values = new HashMap<>();

    @Override
    public OptionalDouble get(String step) {
      Double v = values.get(step);
      return v == null ? OptionalDouble.empty() : OptionalDouble.of(v);
    }

    @Override
    public void put(String step, double value) {
      values.put(step, value);
    }
  }

  @FunctionalInterface
  private interface Step {
    Command command(SystemsCheck check);
  }

  private final BaselineStore baseline;
  private final List<Step> steps = new ArrayList<>();
  private final List<Subsystem> requirements = new ArrayList<>();
  private final Map<String, Result> results = new LinkedHashMap<>();

  public SystemsCheck(BaselineStore baseline) {
    this.baseline = baseline;
  }

  // ---------------------------------------------------------------------------------------------
  // Building the check

  /**
   * The full pit check for this robot, in the order it runs: devices that do not move, then swerve,
   * then the fuel path from intake to shooter, then the hood and the hopper extension.
   */
  public static SystemsCheck standard(
      BaselineStore baseline,
      Drive drive,
      Vision vision,
      Intake intake,
      Indexer indexer,
      Feeder feeder,
      Kicker kicker,
      Shooter shooter,
      Hood hood,
      Extension extension) {
    SystemsCheck check = new SystemsCheck(baseline);
    check.addPresence("Gyro", drive::isGyroConnected);
    for (int i = 0; i < vision.getCameraCount(); i++) {
      int camera = i;
      check.addPresence("Camera " + i, () -> vision.isCameraConnected(camera));
    }
    check.addSwerve(drive);
    check.addIntake(intake);
    check.addRoller("Indexer", indexer, IndexerConstants.FORWARD_VELOCITY);
    check.addRoller("Feeder", feeder, FeederConstants.FEED_VELOCITY);
    check.addRoller("Kicker", kicker, KickerConstants.SHOOT_VELOCITY);
    // A speed the robot really shoots at: the table's 150 in shot, as TuningCommands uses.
    check.addShooter(
        shooter, RPM.of(ShooterMath.getShotParameters(Units.inchesToMeters(150.0)).rpm()));
    check.addHood(hood);
    check.addExtension(extension);
    return check;
  }

  /** A device that has no motion to test, just "is it there". */
  public SystemsCheck addPresence(String name, BooleanSupplier present) {
    addStep(
        name,
        check ->
            Commands.runOnce(
                () ->
                    check.record(
                        name,
                        present.getAsBoolean()
                            ? new Result(Status.PASS, "connected", Double.NaN)
                            : new Result(Status.FAIL, "not detected", Double.NaN))));
    return this;
  }

  /** Spin a roller to {@code speed} and time how long it takes to get within 90% of it. */
  public SystemsCheck addRoller(String name, RollerSubsystem roller, AngularVelocity speed) {
    requirements.add(roller);
    addStep(
        name,
        check ->
            check.timedStep(
                name,
                roller,
                () -> roller.setVelocity(speed),
                roller::stop,
                () -> roller.getVelocity().in(RPM) >= SPEED_REACHED_FRACTION * speed.in(RPM),
                STEP_CAP_SECONDS,
                () ->
                    String.format("%.0f RPM of %.0f", roller.getVelocity().in(RPM), speed.in(RPM)),
                roller::isConnected));
    return this;
  }

  /**
   * Intake roller: open-loop, so check it spins forward to 90% of its free speed at run voltage.
   */
  public SystemsCheck addIntake(Intake intake) {
    requirements.add(intake);
    double expectedRpm =
        IntakeConstants.ROLLER_RPM_PER_VOLT * IntakeConstants.INTAKE_RUN_VOLTAGE.in(Volts);
    addStep(
        "Intake",
        check ->
            check.timedStep(
                "Intake",
                intake,
                intake::runRoller,
                intake::stopRoller,
                () -> intake.getRollerVelocity().in(RPM) >= SPEED_REACHED_FRACTION * expectedRpm,
                STEP_CAP_SECONDS,
                () ->
                    String.format(
                        "%.0f RPM of %.0f", intake.getRollerVelocity().in(RPM), expectedRpm),
                intake::isRollerConnected));
    return this;
  }

  /** Spin the flywheel to {@code speed}. */
  public SystemsCheck addShooter(Shooter shooter, AngularVelocity speed) {
    requirements.add(shooter);
    addStep(
        "Shooter",
        check ->
            check.timedStep(
                "Shooter",
                shooter,
                () -> shooter.setVelocity(speed),
                shooter::stop,
                () -> shooter.getVelocity().in(RPM) >= SPEED_REACHED_FRACTION * speed.in(RPM),
                STEP_CAP_SECONDS,
                () ->
                    String.format("%.0f RPM of %.0f", shooter.getVelocity().in(RPM), speed.in(RPM)),
                shooter::isConnected));
    return this;
  }

  /** Drive the hood to each end of the range it is allowed to use. */
  public SystemsCheck addHood(Hood hood) {
    requirements.add(hood);
    // The soft minimum, not the 50 deg stop: driving into the shallow stop breaks the hood.
    for (Angle target :
        new Angle[] {HoodConstants.HOOD_SOFT_MIN_ANGLE, HoodConstants.MAX_HOOD_ANGLE}) {
      String name = String.format("Hood to %.0f deg", target.in(Degrees));
      addStep(
          name,
          check ->
              check.timedStep(
                  name,
                  hood,
                  () -> hood.setTargetAngleUntrimmed(target),
                  () -> {},
                  () ->
                      Math.abs(hood.getAngle().in(Degrees) - target.in(Degrees))
                          <= HoodConstants.HOOD_ANGLE_TOLERANCE.in(Degrees),
                  STEP_CAP_SECONDS,
                  () -> String.format("at %.1f deg", hood.getAngle().in(Degrees)),
                  hood::isConnected));
    }
    return this;
  }

  /**
   * Push the hopper out (timed, like a match) and pull it back until the limit switch trips. The
   * outward step checks the switch releases; the inward one checks it trips again.
   */
  public SystemsCheck addExtension(Extension extension) {
    requirements.add(extension);
    addStep(
        "Extension out",
        check ->
            check.timedStep(
                "Extension out",
                extension,
                extension::driveOut,
                extension::stopMotor,
                () -> !extension.isRetracted(),
                ExtensionConstants.EXTEND_TIME.in(Seconds),
                () -> extension.isRetracted() ? "limit switch still pressed" : "left the switch",
                extension::isConnected));
    // Finish the extend so the retract has the full travel to cover. Not graded.
    steps.add(check -> extension.extend());
    addStep(
        "Extension in (limit switch)",
        check ->
            check.timedStep(
                "Extension in (limit switch)",
                extension,
                extension::driveIn,
                extension::stopMotor,
                extension::isRetracted,
                RETRACT_TIME_MULTIPLE * ExtensionConstants.EXTEND_TIME.in(Seconds),
                () -> extension.isRetracted() ? "switch pressed" : "switch never pressed",
                extension::isConnected));
    return this;
  }

  /** Swerve steer settles when it is within this of the commanded angle. */
  static final double STEER_TOLERANCE_DEG = 5.0;

  /**
   * Wheel speed for the drive steps. Slow enough to be safe on the cart, fast enough that encoder
   * noise cannot pass for motion: 5% of the 4.83 m/s free speed (TunerConstants.kSpeedAt12Volts).
   */
  static final double SWERVE_TEST_SPEED_MPS =
      0.05 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

  private static final String[] MODULE_NAMES = {"FL", "FR", "BL", "BR"};

  /**
   * All four modules together, graded one by one: steer to 90 deg (a quarter turn, so a steer motor
   * or encoder that is reversed or offset by half a turn cannot pass), then drive forward and
   * reverse, checking each measured wheel speed has the commanded sign. Together because that is
   * how the modules are loaded on the field: in sim, spinning one wheel alone drags the others and
   * reads about half speed. Wheels must be off the ground.
   */
  public SystemsCheck addSwerve(Drive drive) {
    requirements.add(drive);
    Rotation2d quarterTurn = Rotation2d.kCCW_90deg;
    addModulePhase(
        drive,
        " steer",
        (module, name) ->
            timedStep(
                name,
                null,
                () -> module.runRaw(quarterTurn, 0.0),
                () -> module.runRaw(quarterTurn, 0.0), // keep pointing for the drive phases
                () ->
                    Math.abs(module.getAngle().minus(quarterTurn).getDegrees())
                        <= STEER_TOLERANCE_DEG,
                STEP_CAP_SECONDS,
                () -> String.format("at %.0f deg", module.getAngle().getDegrees()),
                module::isConnected));
    for (double sign : new double[] {1.0, -1.0}) {
      double target = sign * SWERVE_TEST_SPEED_MPS;
      addModulePhase(
          drive,
          sign > 0 ? " drive forward" : " drive reverse",
          (module, name) ->
              timedStep(
                  name,
                  null,
                  () -> module.runRaw(quarterTurn, target),
                  () -> module.runRaw(quarterTurn, 0.0),
                  () ->
                      module.getVelocityMetersPerSec() * sign
                          >= SPEED_REACHED_FRACTION * SWERVE_TEST_SPEED_MPS,
                  STEP_CAP_SECONDS,
                  () -> String.format("%.2f m/s of %.2f", module.getVelocityMetersPerSec(), target),
                  module::isConnected));
    }
    steps.add(check -> Commands.runOnce(drive::stop, drive));
    return this;
  }

  /**
   * One graded step per module, run in parallel, holding {@code drive} for the whole phase. A
   * module that finishes early keeps running its hold action until the slowest one is done.
   */
  private void addModulePhase(
      Drive drive, String suffix, BiFunction<Module, String, Command> step) {
    String[] names = new String[MODULE_NAMES.length];
    for (int i = 0; i < names.length; i++) {
      names[i] = "Swerve " + MODULE_NAMES[i] + suffix;
      results.put(names[i], new Result(Status.NOT_RUN, "", Double.NaN));
    }
    steps.add(
        check -> {
          Command[] modules = new Command[names.length];
          for (int i = 0; i < names.length; i++) {
            modules[i] = step.apply(drive.getModule(i), names[i]);
          }
          Command phase = new ParallelCommandGroup(modules);
          phase.addRequirements(drive);
          return phase;
        });
  }

  private void addStep(String name, Function<SystemsCheck, Command> factory) {
    results.put(name, new Result(Status.NOT_RUN, "", Double.NaN));
    steps.add(factory::apply);
  }

  // ---------------------------------------------------------------------------------------------
  // Running

  /** Runs every step in order. Stops early, marking the rest SKIPPED, if the robot is disabled. */
  public Command run() {
    List<Command> sequence = new ArrayList<>();
    sequence.add(Commands.runOnce(this::start));
    for (Step step : steps) {
      sequence.add(step.command(this).onlyWhile(DriverStation::isEnabled));
    }
    sequence.add(Commands.runOnce(this::finish));
    return Commands.either(
            Commands.sequence(sequence.toArray(Command[]::new))
                .finallyDo(this::markUnfinishedSkipped),
            Commands.runOnce(this::refuseOnField),
            () -> !DriverStation.isFMSAttached())
        .withName("SystemsCheck");
  }

  /**
   * Stores the last run's times as the healthy baseline. Refuses if anything failed or was skipped.
   */
  public Command saveBaseline() {
    return Commands.runOnce(
            () -> {
              Status overall = overall();
              if (overall != Status.PASS && overall != Status.WARN) {
                notify(
                    Elastic.NotificationLevel.ERROR,
                    "Baseline not saved",
                    "Run a full check with no failures first (last run: " + overall + ").");
                return;
              }
              int saved = 0;
              for (var entry : results.entrySet()) {
                if (!Double.isNaN(entry.getValue().metric())) {
                  baseline.put(entry.getKey(), entry.getValue().metric());
                  saved++;
                }
              }
              notify(
                  Elastic.NotificationLevel.INFO,
                  "Baseline saved",
                  saved + " timings stored on the roboRIO.");
            })
        .ignoringDisable(true)
        .withName("SystemsCheckSaveBaseline");
  }

  public Map<String, Result> results() {
    return results;
  }

  /** Worst status across all steps. */
  public Status overall() {
    boolean anyRun = false;
    Status worst = Status.PASS;
    for (Result r : results.values()) {
      switch (r.status()) {
        case FAIL -> {
          return Status.FAIL;
        }
        case SKIPPED -> worst = worst == Status.WARN ? Status.WARN : Status.SKIPPED;
        case WARN -> worst = Status.WARN;
        case NOT_RUN, RUNNING -> {
          return anyRun ? Status.SKIPPED : Status.NOT_RUN;
        }
        case PASS -> {}
      }
      anyRun = true;
    }
    return anyRun ? worst : Status.NOT_RUN;
  }

  private void start() {
    results.replaceAll((k, v) -> new Result(Status.NOT_RUN, "", Double.NaN));
    publish();
  }

  private void finish() {
    publish();
    Status overall = overall();
    long failed = results.values().stream().filter(r -> r.status() == Status.FAIL).count();
    long warned = results.values().stream().filter(r -> r.status() == Status.WARN).count();
    notify(
        overall == Status.FAIL
            ? Elastic.NotificationLevel.ERROR
            : overall == Status.WARN
                ? Elastic.NotificationLevel.WARNING
                : Elastic.NotificationLevel.INFO,
        "Systems check: " + overall,
        overall == Status.PASS
            ? "Everything passed."
            : failed + " failed, " + warned + " slower than baseline. See the Pit tab.");
  }

  private void refuseOnField() {
    results.replaceAll((k, v) -> new Result(Status.SKIPPED, "FMS attached", Double.NaN));
    publish();
  }

  private void markUnfinishedSkipped() {
    results.replaceAll(
        (k, v) ->
            v.status() == Status.NOT_RUN || v.status() == Status.RUNNING
                ? new Result(Status.SKIPPED, "robot disabled before this step", Double.NaN)
                : v);
    publish();
  }

  void record(String name, Result result) {
    results.put(name, result);
    publish();
  }

  /**
   * A step that drives a mechanism until {@code reached} or {@code capSeconds}, timing it.
   *
   * @param subsystem required while the step runs; null when an enclosing group holds it
   * @param connected checked first; a disconnected device FAILs without being driven
   * @param state describes where the mechanism got to, for the failure message
   */
  Command timedStep(
      String name,
      Subsystem subsystem,
      Runnable drive,
      Runnable stop,
      BooleanSupplier reached,
      double capSeconds,
      Supplier<String> state,
      BooleanSupplier connected) {
    Timer timer = new Timer();
    double[] reachedAt = {Double.NaN};
    Command driveUntil =
        Commands.run(
                () -> {
                  drive.run();
                  if (Double.isNaN(reachedAt[0]) && reached.getAsBoolean()) {
                    reachedAt[0] = timer.get();
                  }
                },
                subsystem == null ? new Subsystem[0] : new Subsystem[] {subsystem})
            .beforeStarting(
                () -> {
                  timer.restart();
                  reachedAt[0] = Double.NaN;
                  record(name, new Result(Status.RUNNING, "", Double.NaN));
                })
            .until(() -> !Double.isNaN(reachedAt[0]) || timer.get() >= capSeconds)
            .finallyDo(
                interrupted -> {
                  stop.run();
                  if (!interrupted) { // an interrupted step is reported by markUnfinishedSkipped
                    record(name, grade(name, reachedAt[0], capSeconds, state.get()));
                  }
                });
    return Commands.either(
        driveUntil,
        Commands.runOnce(() -> record(name, new Result(Status.FAIL, "disconnected", Double.NaN))),
        connected);
  }

  private Result grade(String name, double reachedAt, double capSeconds, String state) {
    if (Double.isNaN(reachedAt)) {
      return new Result(
          Status.FAIL, String.format("not reached in %.1f s (%s)", capSeconds, state), Double.NaN);
    }
    OptionalDouble base = baseline.get(name);
    if (base.isPresent() && reachedAt > DRIFT_RATIO * base.getAsDouble()) {
      return new Result(
          Status.WARN,
          String.format(
              "%.2f s, baseline %.2f s (%.0f%% slower)",
              reachedAt, base.getAsDouble(), (reachedAt / base.getAsDouble() - 1) * 100),
          reachedAt);
    }
    return new Result(
        Status.PASS,
        base.isPresent()
            ? String.format("%.2f s (baseline %.2f s)", reachedAt, base.getAsDouble())
            : String.format("%.2f s (no baseline yet)", reachedAt),
        reachedAt);
  }

  // ---------------------------------------------------------------------------------------------
  // Dashboard

  /** NetworkTables table the Pit tab reads from. */
  public static final String TABLE = "SystemsCheck";

  /**
   * WPILib alert group the Pit tab shows as its report: FAIL is an error, WARN a warning, and PASS
   * or RUNNING an info line. Elastic's Alerts widget lists errors first.
   */
  public static final String ALERT_GROUP = "Systems Check";

  private final Map<String, Alert[]> stepAlerts = new HashMap<>();

  private void publish() {
    List<String> rows = new ArrayList<>();
    for (var entry : results.entrySet()) {
      String name = entry.getKey();
      Result r = entry.getValue();
      rows.add(String.format("%-7s %s  %s", r.status(), name, r.detail()).trim());
      Logger.recordOutput(TABLE + "/Steps/" + name + "/Status", r.status().name());
      Logger.recordOutput(TABLE + "/Steps/" + name + "/Detail", r.detail());
      if (!Double.isNaN(r.metric())) {
        Logger.recordOutput(TABLE + "/Steps/" + name + "/Seconds", r.metric());
      }
      showAlert(name, r);
    }
    Status overall = overall();
    Logger.recordOutput(TABLE + "/Overall", overall.name());
    Logger.recordOutput(TABLE + "/Report", rows.toArray(String[]::new));
    Logger.recordOutput(TABLE + "/Passed", overall == Status.PASS);
    SmartDashboard.putString(Dashboard.PIT + "Result", overall.name());
  }

  private void showAlert(String name, Result r) {
    Alert[] alerts =
        stepAlerts.computeIfAbsent(
            name,
            k ->
                new Alert[] {
                  new Alert(ALERT_GROUP, "", AlertType.kError),
                  new Alert(ALERT_GROUP, "", AlertType.kWarning),
                  new Alert(ALERT_GROUP, "", AlertType.kInfo)
                });
    int active =
        switch (r.status()) {
          case FAIL -> 0;
          case WARN, SKIPPED -> 1;
          case PASS, RUNNING -> 2;
          case NOT_RUN -> -1;
        };
    String text =
        r.status() == Status.RUNNING
            ? name + ": running"
            : name + ": " + r.status() + (r.detail().isEmpty() ? "" : " - " + r.detail());
    for (int i = 0; i < alerts.length; i++) {
      alerts[i].setText(text);
      alerts[i].set(i == active);
    }
  }

  private static void notify(Elastic.NotificationLevel level, String title, String description) {
    Elastic.sendNotification(
        new Elastic.Notification(level, title, description).withDisplaySeconds(8.0));
  }

  /** Every subsystem any step drives; for callers that need to cancel conflicting commands. */
  public List<Subsystem> getRequirements() {
    return requirements;
  }
}
