package frc.robot.auton;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
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
   * How long a "shootHub" Choreo event runs the full shot (spin-up + auto-feed).
   *
   * <p>Measured from {@code LeftTriggerShotDiagnosticTest}: the flywheel reaches setpoint at ~0.5
   * s, but the drive's aim only converges inside {@code AIM_TOLERANCE_RAD} at ~1.5-2.0 s, and
   * feeding cannot begin until it does. A 0.5 s {@link
   * frc.robot.subsystems.shooter.ShooterConstants#FEED_INTAKE_PULSE} then has to fit after that.
   * The previous 1.0 s value expired before a single fuel was fed, so auto shoot events scored
   * nothing.
   */
  public static final double SHOOT_HUB_EVENT_SECONDS = 3.0;

  /**
   * Upper bound on one auto volley. A volley normally ends as soon as the hopper is empty; this
   * only stops a jammed or mis-aimed robot from burning the rest of AUTO. Same measured budget as
   * {@link #SHOOT_HUB_EVENT_SECONDS} (aim settle + feed) plus 1 s to empty a full hopper.
   */
  public static final double VOLLEY_TIMEOUT_SECONDS = 4.0;

  /**
   * Trench autos start the flywheel this long before the first sweep reaches its shooting stop. On
   * the robot it took about 1.7 s from rest to shooting speed; the volley waits for the rest.
   */
  public static final double SPIN_UP_LEAD_SECONDS = 1.5;

  private final Drive drive;
  private final Intake intake;
  private final Extension extension;
  private final Hood hood;
  private final Shooter shooter;
  private final Kicker kicker;
  private final Feeder feeder;
  private final Indexer indexer;
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
    this.intake = intake;
    this.extension = extension;
    this.hood = hood;
    this.shooter = shooter;
    this.kicker = kicker;
    this.feeder = feeder;
    this.indexer = indexer;
    autoFactory =
        new AutoFactory(drive::getPose, drive::setPose, drive::followChoreoSample, true, drive);
    // Event bindings into autoFactory are registered if an autonomous routine uses event markers
    // on its trajectory. LeftTrench is composed in code with deadlines, and Test has no markers;
    // binding non-existent markers causes ChoreoLib to log "Event Markers not found" on startup.
    // Call configureEventBindings(...) if markers are placed in the Choreo GUI.
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

  /**
   * Left Trench two-cycle auto: sweep the neutral zone, stop aimed at the hub and shoot, sweep
   * again, stop aimed at the hub and shoot. Both passes under the trench are centred on its opening
   * (see {@code TrenchClearanceTest}).
   */
  public Command leftTrench() {
    return trenchAuto("LeftTrench", ChoreoTraj.Left_Trench$0, ChoreoTraj.Left_Trench$1);
  }

  /**
   * {@link #leftTrench} mirrored across the field's long axis (y to fieldWidth - y, heading
   * negated), for when a partner takes the left trench.
   */
  public Command rightTrench() {
    return trenchAuto("RightTrench", ChoreoTraj.Right_Trench$0, ChoreoTraj.Right_Trench$1);
  }

  /**
   * Preload-only fallback: start with the rear bumper against the middle of the hub's alliance face
   * (a repeatable placement by touch, on the ROBOT STARTING LINE), back straight away 1.41 m to put
   * the shooter 90 in from the hub centre, and shoot all the preloaded fuel. The flywheel spins up
   * while the robot drives, so feeding starts as soon as it stops.
   */
  public Command centerPreload() {
    AutoRoutine routine = autoFactory.newRoutine("CenterPreload");
    AutoTrajectory backUp = ChoreoTraj.Center_Preload.asAutoTraj(routine);

    Command drivePlan =
        Commands.sequence(
            backUp.resetOdometry(),
            Commands.runOnce(drive::resetChoreoControllers, drive),
            backUp.cmd(),
            volley(),
            drive.runOnce(drive::stop));

    routine
        .active()
        .onTrue(Commands.deadline(drivePlan, ShootCommands.spinUp(drive, shooter, hood, kicker)));

    return routine.cmd().withName("Auto_CenterPreload");
  }

  /**
   * Preload plus the depot: start under the left trench, in the same spot as {@link #leftTrench}
   * but facing the alliance wall, drive out from under it, then to the tower end of the depot,
   * sweep up along the wall intake-first at 1.5 m/s, then leave past the far end and stop to the
   * left of the hub with the shooter facing it and shoot the preload and everything collected in
   * one volley. The depot is swept from the side because entering it head-on caused problems on the
   * robot, and the robot leaves past the far end so it doesn't cross the depot again.
   *
   * <p>Not yet run on the robot. In maple-sim the robot clips the tower on the way to the depot
   * (see {@code AutoScoringDepotHarness}), so watch that spot on the first run.
   */
  public Command centerDepot() {
    AutoRoutine routine = autoFactory.newRoutine("CenterDepot");
    AutoTrajectory toDepot = ChoreoTraj.Center_Depot.asAutoTraj(routine);

    Command drivePlan =
        Commands.sequence(
            toDepot.resetOdometry(),
            Commands.runOnce(drive::resetChoreoControllers, drive),
            Commands.deadline(toDepot.cmd(), extension.extend(), intakeWhileExtended()),
            volley(),
            drive.runOnce(drive::stop));

    // Same delayed spin-up as the trench autos, to keep the flywheel's spin-up current away from
    // the drive's launch off the hub.
    double spinUpDelaySeconds =
        Math.max(0.0, ChoreoTraj.Center_Depot.totalTimeSecs() - SPIN_UP_LEAD_SECONDS);
    routine
        .active()
        .onTrue(
            Commands.deadline(
                drivePlan,
                Commands.waitSeconds(spinUpDelaySeconds)
                    .andThen(ShootCommands.spinUp(drive, shooter, hood, kicker))));

    return routine.cmd().withName("Auto_CenterDepot");
  }

  /**
   * Two sweeps with a volley after each. The path is split at the first shooting stop so each half
   * is its own trajectory; the second half only starts once the first volley has been fired. Intake
   * runs for each whole sweep so every fuel the path passes over is collected. The flywheel and
   * hood start tracking the shot 1.5 s before the first stop (see SPIN_UP_LEAD_SECONDS) and keep
   * tracking it for the rest of the routine.
   */
  private Command trenchAuto(String name, ChoreoTraj firstSweep, ChoreoTraj secondSweep) {
    AutoRoutine routine = autoFactory.newRoutine(name);
    AutoTrajectory sweep1 = firstSweep.asAutoTraj(routine);
    AutoTrajectory sweep2 = secondSweep.asAutoTraj(routine);

    Command drivePlan =
        Commands.sequence(
            sweep1.resetOdometry(),
            Commands.runOnce(drive::resetChoreoControllers, drive),
            Commands.deadline(sweep1.cmd(), extension.extend(), intakeWhileExtended()),
            volley(),
            // The volley's creepIn pulls the hopper in and clears fullyExtended, so the intake
            // stays gated off unless the hopper is pushed back out for the next sweep.
            Commands.deadline(sweep2.cmd(), extension.extend(), intakeWhileExtended()),
            volley(),
            // The aim command leaves its last module setpoints behind when it is cancelled;
            // without this the robot kept drifting (0.9 m, 45 deg) until the end of AUTO.
            drive.runOnce(drive::stop));

    // Spin up only for the end of the first sweep, then keep the flywheel going for the rest of
    // the routine. Spinning up from rest at the start of AUTO (4 Vortexes, ~260 A) while the drive
    // accelerated browned the robot out on the field (6.9 V, BrownedOut set), and the timed hopper
    // extend ran out of time at ~87%. Holding speed afterwards takes only 30-45 A.
    double spinUpDelaySeconds = Math.max(0.0, firstSweep.totalTimeSecs() - SPIN_UP_LEAD_SECONDS);
    routine
        .active()
        .onTrue(
            Commands.deadline(
                drivePlan,
                Commands.waitSeconds(spinUpDelaySeconds)
                    .andThen(ShootCommands.spinUp(drive, shooter, hood, kicker))));

    return routine.cmd().withName("Auto_" + name);
  }

  /**
   * Aim in place and feed until the hopper is empty, capped so a jam can't stall the auto. The real
   * robot cannot count fuel, so there every volley runs the full timeout. Needs {@link
   * ShootCommands#spinUp} running alongside it.
   */
  private Command volley() {
    return Commands.deadline(
            Commands.waitUntil(intake::isKnownEmpty).withTimeout(VOLLEY_TIMEOUT_SECONDS),
            DriveCommands.joystickDriveAimAtTarget(drive, () -> 0.0, () -> 0.0),
            ShootCommands.autoFeed(
                ShootCommands.readyToFeed(drive, shooter, hood),
                feeder,
                indexer,
                intake,
                extension))
        .withName("Volley");
  }

  private Command intakeWhileExtended() {
    return intake.intake(extension::isFullyExtended);
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
