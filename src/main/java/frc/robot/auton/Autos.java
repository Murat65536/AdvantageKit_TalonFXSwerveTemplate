package frc.robot.auton;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
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

/** Centralized autonomous routine definitions backed by Choreo. */
public class Autos {
  private final Drive drive;
  private final AutoFactory autoFactory;

  public Autos(Drive drive, Intake intake, Extension extension, Hood hood, Shooter shooter) {
    this.drive = drive;
    autoFactory =
        new AutoFactory(drive::getPose, drive::setPose, drive::followChoreoSample, true, drive);
    configureEventBindings(intake, extension, hood, shooter);
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
                autoTrajectory.cmd()));

    return routine.cmd().withName("Auto_" + trajectory.name());
  }

  private void configureEventBindings(
      Intake intake, Extension extension, Hood hood, Shooter shooter) {
    autoFactory
        .bind("extend", extension.extend().withTimeout(0.4))
        .bind("retract", extension.retract().withTimeout(0.4))
        .bind("intake", intake.intake(extension::isFullyExtended).withTimeout(1.2))
        .bind("outtake", intake.outtake().withTimeout(0.6))
        .bind(
            "collect",
            Commands.parallel(
                extension.extend().withTimeout(0.35),
                intake.intake(extension::isFullyExtended).withTimeout(1.25)))
        .bind(
            "shootHub",
            shooter
                .shootAtHub(drive::getPose, drive::getFieldRelativeChassisSpeeds, hood)
                .withTimeout(1.0));
  }
}
