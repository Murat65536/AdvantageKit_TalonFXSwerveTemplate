package frc.robot.auton;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.choreo.ChoreoTraj;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.extension.Extension;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import java.util.Optional;

/**
 * Centralized autonomous routine definitions backed by Choreo. Trajectories are followed with a
 * clock-governed command (see {@link GovernedTrajectoryCommand}) rather than ChoreoLib's wall-clock
 * {@code AutoFactory} playback, so the follower can pause the trajectory and recover from drift
 * instead of running away.
 */
public class Autos {
  private final Drive drive;
  private final Map<String, Command> eventCommands;

  public Autos(Drive drive, Intake intake, Extension extension, Hood hood, Shooter shooter) {
    this.drive = drive;
    this.eventCommands = buildEventCommands(intake, extension, hood, shooter);
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
    Optional<Trajectory<SwerveSample>> loaded = Choreo.loadTrajectory(trajectory.name());
    if (loaded.isEmpty()) {
      new Alert("Choreo trajectory '" + trajectory.name() + "' failed to load.", AlertType.kError)
          .set(true);
      return Commands.none();
    }
    return new GovernedTrajectoryCommand(loaded.get(), drive, eventCommands);
  }

  /** Maps Choreo event-marker names to the commands they trigger when the path reaches them. */
  private Map<String, Command> buildEventCommands(
      Intake intake, Extension extension, Hood hood, Shooter shooter) {
    return Map.of(
        "extend",
        extension.extend().withTimeout(0.4),
        "retract",
        extension.retract().withTimeout(0.4),
        "intake",
        intake.intake(extension::isFullyExtended).withTimeout(1.2),
        "outtake",
        intake.outtake().withTimeout(0.6),
        "collect",
        Commands.parallel(
            extension.extend().withTimeout(0.35),
            intake.intake(extension::isFullyExtended).withTimeout(1.25)),
        "shootHub",
        shooter
            .shootAtHub(drive::getPose, drive::getFieldRelativeChassisSpeeds, hood)
            .withTimeout(1.0));
  }
}
