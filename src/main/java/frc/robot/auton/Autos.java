package frc.robot.auton;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ShootCommands;
import frc.robot.generated.choreo.ChoreoTraj;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.extension.Extension;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import java.util.Comparator;
import java.util.List;

/** Centralized autonomous routine definitions backed by Choreo. */
public class Autos {
  /**
   * How long a "shootHub" Choreo event runs the full shot (spin-up + auto-feed). The previous
   * flywheel-only binding used 1.0 s; with the feed path this needs to cover spin-up plus feeding.
   */
  public static final double SHOOT_HUB_EVENT_SECONDS = 1.0;

  private final Drive drive;
  private final AutoFactory autoFactory;

  public Autos(
      Drive drive,
      Intake intake,
      Extension extension,
      Hood hood,
      Shooter shooter,
      Kicker kicker,
      Feeder feeder,
      Indexer indexer) {
    this.drive = drive;
    autoFactory =
        new AutoFactory(drive::getPose, drive::setPose, drive::followChoreoSample, true, drive);
    configureEventBindings(intake, extension, hood, shooter, kicker, feeder, indexer);
  }

  /** Returns all top-level generated trajectories available for dashboard selection. */
  public List<ChoreoTraj> getAvailableTrajectories() {
    return ChoreoTraj.ALL_TRAJECTORIES.values().stream()
        .filter((trajectory) -> trajectory.segment().isEmpty())
        .sorted(Comparator.comparing(ChoreoTraj::name))
        .toList();
  }

  /** Builds a full autonomous command for a single Choreo trajectory. */
  public Command buildTrajectoryAuto(ChoreoTraj trajectory) {
    AutoRoutine routine = autoFactory.newRoutine(trajectory.name());
    AutoTrajectory autoTrajectory = trajectory.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoTrajectory.resetOdometry(),
                Commands.runOnce(drive::resetChoreoControllers, drive),
                autoTrajectory.cmd(),
                // Hold the endpoint after the trajectory ends so the position controllers settle
                // the
                // robot onto it instead of coasting past (ChoreoLib's cmd() stops correcting once
                // the trajectory time elapses).
                drive.run(
                    () ->
                        drive.holdPose(autoTrajectory.getFinalPose().orElseGet(drive::getPose)))));

    return routine.cmd().withName("Auto_" + trajectory.name());
  }

  private void configureEventBindings(
      Intake intake,
      Extension extension,
      Hood hood,
      Shooter shooter,
      Kicker kicker,
      Feeder feeder,
      Indexer indexer) {
    autoFactory
        // extend() is timed and retract() ends at the limit switch; timeouts are safety caps.
        .bind("extend", extension.extend().withTimeout(1.0))
        .bind("retract", extension.retract().withTimeout(2.0))
        .bind("intake", intake.intake(extension::isFullyExtended).withTimeout(1.2))
        .bind("outtake", intake.outtake().withTimeout(0.6))
        .bind(
            "collect",
            Commands.parallel(
                extension.extend().withTimeout(1.0),
                intake.intake(extension::isFullyExtended).withTimeout(1.25)))
        .bind(
            "shootHub",
            ShootCommands.shootAtTarget(
                    drive, shooter, hood, kicker, feeder, indexer, intake, extension)
                .withTimeout(SHOOT_HUB_EVENT_SECONDS));
  }
}
