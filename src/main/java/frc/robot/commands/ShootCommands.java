package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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
   */
  public static Command shootAtTarget(
      Drive drive,
      Shooter shooter,
      Hood hood,
      Kicker kicker,
      Feeder feeder,
      Indexer indexer,
      Intake intake,
      Extension extension) {
    BooleanSupplier ready =
        () -> {
          boolean flywheelReady = shooter.atSetpoint();
          boolean hoodReady = hood.atSetpoint();
          boolean aimed = ShooterMath.isAimed(drive.getPose());
          boolean solved = shooter.hasShotSolution();
          Logger.recordOutput("Shoot/FlywheelReady", flywheelReady);
          Logger.recordOutput("Shoot/HoodReady", hoodReady);
          Logger.recordOutput("Shoot/Aimed", aimed);
          Logger.recordOutput(
              "Shoot/AimErrorDeg", Math.toDegrees(ShooterMath.getAimErrorRad(drive.getPose())));
          return flywheelReady && hoodReady && aimed && solved;
        };

    return Commands.parallel(
            shooter.shootAtTarget(drive::getPose, drive::getFieldRelativeChassisSpeeds, hood),
            kicker.shoot(),
            autoFeed(ready, feeder, indexer, intake, extension))
        .withName("ShootAtTarget");
  }

  /**
   * Runs the feed path (indexer + feeder, with an initial intake pulse and the hopper creeping
   * inward) whenever {@code ready} is true. Feeding stops only after {@code ready} has been false
   * for {@link frc.robot.subsystems.shooter.ShooterConstants#FEED_STOP_DEBOUNCE}.
   */
  public static Command autoFeed(
      BooleanSupplier ready, Feeder feeder, Indexer indexer, Intake intake, Extension extension) {
    Debouncer stopDebounce = new Debouncer(FEED_STOP_DEBOUNCE.in(Seconds), DebounceType.kFalling);
    BooleanSupplier debouncedReady = () -> stopDebounce.calculate(ready.getAsBoolean());

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
        .beforeStarting(() -> stopDebounce.calculate(false))
        .finallyDo(() -> Logger.recordOutput("Shoot/Feeding", false))
        .withName("AutoFeed");
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
