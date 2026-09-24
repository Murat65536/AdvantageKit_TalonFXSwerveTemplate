package frc.robot.commands;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterMath;
import frc.robot.util.WelfordAccumulator;
import org.littletonrobotics.junction.Logger;

/**
 * Phase-2 tuning aid: holds a fixed hood angle + flywheel RPM setpoint, waits for both loops to
 * settle, then streams the steady-state tracking error into a {@link WelfordAccumulator} for a
 * fixed capture window and logs the resulting stddevs.
 *
 * <p>These stddevs are the two real numbers {@code tools/shot_table_optimizer.py} needs
 * (--sigma-angle-deg, --sigma-v-mps) -- replacing every guessed tolerance in the sim-only Phase 1
 * optimizer with something measured on the actual hardware. Run from the dashboard autoChooser
 * (robot disabled-safe is NOT guaranteed -- run enabled, stationary, propped up or on blocks) and
 * read the printed sigma values off the console / AdvantageScope after {@code Tuning/Capture...}
 * finishes.
 */
public final class TuningCommands {
  private TuningCommands() {}

  private static final double SETTLE_SECONDS = 2.0;
  private static final double CAPTURE_SECONDS = 5.0;
  private static final double SPIN_UP_TIMEOUT_SECONDS = 5.0;
  private static final double DIP_CAPTURE_SECONDS = 1.5;

  public static Command captureTrackingNoise(
      Shooter shooter, Hood hood, AngularVelocity targetRpm, Angle targetHoodAngle) {
    WelfordAccumulator rpmError = new WelfordAccumulator();
    WelfordAccumulator angleError = new WelfordAccumulator();

    Command settle =
        Commands.run(
                () -> {
                  shooter.setVelocity(targetRpm);
                  hood.setTargetAngle(targetHoodAngle);
                },
                shooter,
                hood)
            .withTimeout(SETTLE_SECONDS)
            .withName("TuningSettle");

    Command capture =
        Commands.run(
                () -> {
                  shooter.setVelocity(targetRpm);
                  hood.setTargetAngle(targetHoodAngle);
                  double rpmErr = shooter.getVelocity().in(RPM) - targetRpm.in(RPM);
                  double angleErr = hood.getAngle().in(Degrees) - targetHoodAngle.in(Degrees);
                  rpmError.add(rpmErr);
                  angleError.add(angleErr);
                  Logger.recordOutput("Tuning/InstantRpmErrorRpm", rpmErr);
                  Logger.recordOutput("Tuning/InstantAngleErrorDeg", angleErr);
                },
                shooter,
                hood)
            .withTimeout(CAPTURE_SECONDS)
            .withName("TuningCapture");

    return Commands.sequence(settle, capture)
        .andThen(
            Commands.runOnce(
                () -> {
                  double sigmaRpm = rpmError.getStdDev();
                  double sigmaAngleDeg = angleError.getStdDev();
                  // Convert RPM stddev to exit-velocity stddev (m/s) using the same conversion
                  // ShooterMath uses, so the two numbers plug directly into
                  // shot_table_optimizer.py's --sigma-v-mps.
                  double sigmaVMps =
                      RPM.of(sigmaRpm).in(RadiansPerSecond)
                          * EXIT_VELOCITY_PER_FLYWHEEL_RAD_PER_SEC;

                  Logger.recordOutput("Tuning/Result/SampleCount", rpmError.getCount());
                  Logger.recordOutput("Tuning/Result/MeanRpmErrorRpm", rpmError.getMean());
                  Logger.recordOutput("Tuning/Result/SigmaRpm", sigmaRpm);
                  Logger.recordOutput("Tuning/Result/SigmaExitVelocityMps", sigmaVMps);
                  Logger.recordOutput("Tuning/Result/MeanAngleErrorDeg", angleError.getMean());
                  Logger.recordOutput("Tuning/Result/SigmaAngleDeg", sigmaAngleDeg);

                  String summary =
                      String.format(
                          "TuningCapture done (n=%d, %.1fs): "
                              + "hood mean=%.3fdeg sigma=%.4fdeg | "
                              + "flywheel mean=%.1fRPM sigma=%.2fRPM (%.4f m/s exit). "
                              + "Run: python3 tools/shot_table_optimizer.py "
                              + "--sigma-angle-deg %.4f --sigma-v-mps %.4f --output shot_table_measured.json",
                          rpmError.getCount(),
                          CAPTURE_SECONDS,
                          angleError.getMean(),
                          sigmaAngleDeg,
                          rpmError.getMean(),
                          sigmaRpm,
                          sigmaVMps,
                          sigmaAngleDeg,
                          sigmaVMps);
                  System.out.println(summary);
                  DriverStation.reportWarning(summary, false);
                }))
        .finallyDo(
            () -> {
              shooter.stop();
            })
        .withName("TuningCaptureTrackingNoise");
  }

  /**
   * Convenience overload that holds the shot the table actually commands at 150 in.
   *
   * <p>The RPM and hood angle are read from {@link ShooterMath} rather than hardcoded, so
   * re-deriving the table cannot leave this sampling the hood at an angle the robot no longer uses.
   */
  public static Command captureTrackingNoiseAtMidRangeShot(Shooter shooter, Hood hood) {
    var shot = ShooterMath.getShotParameters(Units.inchesToMeters(150.0));
    return captureTrackingNoise(shooter, hood, RPM.of(shot.rpm()), Degrees.of(shot.angleDeg()))
        .withName("TuningCaptureTrackingNoiseMidRange");
  }

  /**
   * Holds a hood angle and flywheel setpoint, then feeds fuel and records the flywheel's MINIMUM
   * measured RPM during the shot -- the speed the ball actually leaves at.
   *
   * <p>This exists because the flywheel dips when a ball passes through it. The setpoint is not the
   * exit speed, and feeding the setpoint into {@code tools/fit_slip_factor.py} folds the dip into
   * the slip factor, producing a fit that will not transfer to a different feed rate. Use the
   * printed {@code rpmAtExit} as the MEASURED_RPM argument.
   *
   * <p>Run this stationary and pointed somewhere safe. Measure where the fuel first hits the floor
   * and pair that distance with the printed RPM.
   *
   * @param holdAngle hood angle to hold; pick one whose floor range fits your space (at slip ~0.58,
   *     60 deg / 2500 RPM lands ~4.9 m out)
   */
  public static Command captureShotDip(
      Shooter shooter,
      Hood hood,
      Kicker kicker,
      Feeder feeder,
      Indexer indexer,
      Intake intake,
      AngularVelocity targetRpm,
      Angle holdAngle) {
    // Mutable holders: the lambdas below run on the scheduler thread across several ticks.
    double[] minRpm = {Double.POSITIVE_INFINITY};
    double[] preShotRpm = {0.0};
    boolean[] feeding = {false};

    Command spinUp =
        Commands.run(
                () -> {
                  shooter.setVelocity(targetRpm);
                  hood.setTargetAngle(holdAngle);
                },
                shooter,
                hood)
            .until(() -> shooter.atSetpoint() && hood.atSetpoint())
            .withTimeout(SPIN_UP_TIMEOUT_SECONDS)
            .withName("DipSpinUp");

    Command fire =
        Commands.run(
                () -> {
                  shooter.setVelocity(targetRpm);
                  hood.setTargetAngle(holdAngle);
                  double rpm = shooter.getVelocity().in(RPM);
                  if (!feeding[0]) {
                    preShotRpm[0] = rpm;
                    feeding[0] = true;
                  }
                  minRpm[0] = Math.min(minRpm[0], rpm);
                  Logger.recordOutput("Tuning/Dip/InstantRpm", rpm);
                },
                shooter,
                hood)
            .withTimeout(DIP_CAPTURE_SECONDS)
            .alongWith(kicker.shoot(), feeder.feed(), indexer.index(), intake.intake())
            .withName("DipFire");

    return Commands.sequence(spinUp, fire)
        .andThen(
            Commands.runOnce(
                () -> {
                  double dip = preShotRpm[0] - minRpm[0];
                  Logger.recordOutput("Tuning/Dip/SetpointRpm", targetRpm.in(RPM));
                  Logger.recordOutput("Tuning/Dip/PreShotRpm", preShotRpm[0]);
                  Logger.recordOutput("Tuning/Dip/RpmAtExit", minRpm[0]);
                  Logger.recordOutput("Tuning/Dip/DipRpm", dip);

                  String summary =
                      String.format(
                          "ShotDip: setpoint=%.0f RPM, pre-shot=%.0f, rpmAtExit=%.0f (dip %.0f RPM,"
                              + " %.1f%%), hood=%.1f deg.%n"
                              + "  Measure where the fuel first hit the floor, then run:%n"
                              + "  python3 tools/fit_slip_factor.py range --shot %.1f,%.0f,<RANGE_M>",
                          targetRpm.in(RPM),
                          preShotRpm[0],
                          minRpm[0],
                          dip,
                          preShotRpm[0] > 0 ? dip / preShotRpm[0] * 100.0 : 0.0,
                          holdAngle.in(Degrees),
                          holdAngle.in(Degrees),
                          minRpm[0]);
                  System.out.println(summary);
                  DriverStation.reportWarning(summary, false);
                }))
        .finallyDo(shooter::stop)
        .withName("TuningCaptureShotDip");
  }
}
