package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.extension.Extension;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterMath;
import frc.robot.subsystems.shooter.ShooterMath.ShotSolution;
import frc.robot.util.HubShiftUtil;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/** Coordinated shooting: flywheel + hood aiming, kicker, and the automatic feed path. */
public final class ShootCommands {
  private ShootCommands() {}

  /**
   * Full shot: aim the flywheel/hood at the target, spin the kicker, and automatically feed fuel
   * whenever the flywheel and hood are at their setpoints, a shot solution exists, and the drive is
   * pointed at the target.
   *
   * <p>The kicker is deliberately NOT part of the feed gate. It is an injection roller started by
   * this same command, and it dips on every shot, so gating the feed on it reaching a +/-50 RPM
   * tolerance deadlocks the sequence instead of protecting it.
   *
   * <p>Does not control the drive; pair with {@link DriveCommands#joystickDriveAimAtTarget}.
   *
   * @param ignoreHubSchedule when true, feed even if our hub looks inactive (see {@link
   *     #hubAllowsFeed}); the driver's escape hatch for wrong game data or match time.
   */
  public static Command shootAtTarget(
      Drive drive,
      Shooter shooter,
      Hood hood,
      Kicker kicker,
      Feeder feeder,
      Indexer indexer,
      Intake intake,
      Extension extension,
      BooleanSupplier ignoreHubSchedule) {
    return Commands.parallel(
            spinUp(drive, shooter, hood, kicker),
            autoFeed(
                readyToFeed(drive, shooter, hood, ignoreHubSchedule),
                feeder,
                indexer,
                intake,
                extension))
        .withName("ShootAtTarget");
  }

  /** {@link #shootAtTarget} that respects the hub schedule. */
  public static Command shootAtTarget(
      Drive drive,
      Shooter shooter,
      Hood hood,
      Kicker kicker,
      Feeder feeder,
      Indexer indexer,
      Intake intake,
      Extension extension) {
    return shootAtTarget(
        drive, shooter, hood, kicker, feeder, indexer, intake, extension, () -> false);
  }

  /**
   * Track the shot continuously (flywheel speed + hood angle for the current pose and velocity) and
   * run the kicker, without feeding. Run this while driving so a later {@link #autoFeed} starts
   * with the flywheel already at speed instead of spinning up from zero.
   */
  public static Command spinUp(Drive drive, Shooter shooter, Hood hood, Kicker kicker) {
    return Commands.parallel(
            shooter.shootAtTarget(drive::getPose, drive::getFieldRelativeChassisSpeeds, hood),
            kicker.shoot())
        .withName("SpinUp");
  }

  /**
   * The feed gate: hood at setpoint, a shot solution exists, the drive is aimed, and the fuel would
   * land while our hub counts it. Needs {@link #spinUp} (or {@link #shootAtTarget}) running, which
   * supplies the solution.
   *
   * <p>The flywheel speed is logged but not gated on, like the kicker: every shot dips it below the
   * setpoint band, and on the robot it stayed low enough after the first shot that the rest of the
   * hopper never fed.
   */
  public static BooleanSupplier readyToFeed(
      Drive drive, Shooter shooter, Hood hood, BooleanSupplier ignoreHubSchedule) {
    return () -> {
      boolean flywheelReady = shooter.atSetpoint();
      boolean hoodReady = hood.atSetpoint();
      boolean aimed = ShooterMath.isAimed(drive.getPose());
      boolean solved = shooter.hasShotSolution();
      boolean ignoreHub = ignoreHubSchedule.getAsBoolean();
      boolean hubCounts = hubAllowsFeed(shooter.getShotSolution(), ignoreHub);
      Logger.recordOutput("Shoot/FlywheelReady", flywheelReady);
      Logger.recordOutput("Shoot/HoodReady", hoodReady);
      Logger.recordOutput("Shoot/Aimed", aimed);
      Logger.recordOutput("Shoot/HubCounts", hubCounts);
      Logger.recordOutput("Shoot/IgnoreHubSchedule", ignoreHub);
      Logger.recordOutput("Hub/MatchTime", DriverStation.getMatchTime());
      Logger.recordOutput(
          "Shoot/AimErrorDeg", Math.toDegrees(ShooterMath.getAimErrorRad(drive.getPose())));
      return hoodReady && aimed && solved && hubCounts;
    };
  }

  /** {@link #readyToFeed} that respects the hub schedule. */
  public static BooleanSupplier readyToFeed(Drive drive, Shooter shooter, Hood hood) {
    return readyToFeed(drive, shooter, hood, () -> false);
  }

  /**
   * False only when fuel fired now would land in our inactive hub and score nothing (Game Manual
   * TU22 section 6.4.1). Uses the shot's own time of flight, so a shot released just before the hub
   * activates still goes. With no solution the other gates already block, so this does not.
   */
  public static boolean hubAllowsFeed(ShotSolution solution, boolean ignoreHubSchedule) {
    if (ignoreHubSchedule || solution == null) {
      return true;
    }
    return HubShiftUtil.wouldShotCount(solution.timeOfFlight().in(Seconds));
  }

  /**
   * Runs the feed path (indexer + feeder, with an initial intake pulse and the hopper creeping
   * inward) whenever {@code ready} is true. Feeding stops only after {@code ready} has been false
   * for {@link frc.robot.subsystems.shooter.ShooterConstants#FEED_STOP_DEBOUNCE}.
   */
  public static Command autoFeed(
      BooleanSupplier ready, Feeder feeder, Indexer indexer, Intake intake, Extension extension) {
    FeedHold hold = new FeedHold(FEED_STOP_DEBOUNCE.in(Seconds));
    BooleanSupplier debouncedReady = () -> hold.calculate(ready.getAsBoolean());

    Command feedGroup =
        Commands.parallel(
                Commands.run(() -> Logger.recordOutput("Shoot/Feeding", true)),
                indexer.index(),
                feeder.feed(),
                intake.intake().withTimeout(FEED_INTAKE_PULSE.in(Seconds)),
                extension.creepIn())
            .withName("Feed");

    return Commands.repeatingSequence(
            Commands.run(() -> Logger.recordOutput("Shoot/Feeding", false)).until(debouncedReady),
            feedGroup.until(() -> !debouncedReady.getAsBoolean()))
        .beforeStarting(hold::reset)
        .finallyDo(() -> Logger.recordOutput("Shoot/Feeding", false))
        .withName("AutoFeed");
  }

  /**
   * True while {@code ready}, and for {@code holdSeconds} after it was last true.
   *
   * <p>Not a WPILib {@code Debouncer}: a kFalling Debouncer starts out TRUE and stays true until
   * its input has been false for the whole window, and it cannot be reset to false. Built less than
   * the window before it starts, or re-pressed less than the window after a shot, it fed fuel
   * blind. This starts false on every run because {@link #reset} runs when the command starts.
   */
  private static final class FeedHold {
    private final double holdSeconds;
    private double lastReadySeconds;

    FeedHold(double holdSeconds) {
      this.holdSeconds = holdSeconds;
      reset();
    }

    void reset() {
      lastReadySeconds = Double.NEGATIVE_INFINITY;
    }

    boolean calculate(boolean ready) {
      double now = Timer.getFPGATimestamp();
      if (ready) {
        lastReadySeconds = now;
      }
      return now - lastReadySeconds < holdSeconds;
    }
  }

  /** Reverse the whole fuel path to clear a jam. Stops when released. */
  public static Command unjam(
      Shooter shooter, Kicker kicker, Feeder feeder, Indexer indexer, Intake intake) {
    return Commands.parallel(
            shooter.reverse(),
            kicker.reverse(),
            feeder.reverse(),
            indexer.reverse(),
            intake.outtake())
        .withName("Unjam");
  }
}
