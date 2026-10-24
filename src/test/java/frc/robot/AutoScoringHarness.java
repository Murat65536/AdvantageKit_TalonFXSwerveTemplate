package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.util.ChoreoAllianceFlipUtil;
import com.ctre.phoenix6.unmanaged.Unmanaged;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auton.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFXSim;
import frc.robot.subsystems.extension.Extension;
import frc.robot.subsystems.extension.ExtensionIOSim;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerConstants;
import frc.robot.subsystems.rollers.RollerIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSim;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.LockSupport;
import java.util.function.Function;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.junit.jupiter.api.Test;

/**
 * Runs a complete autonomous period on the real robot stack in maple-sim and measures the only
 * thing auto is scored on: fuel in the hub. Game manual 6.5: AUTO fuel is assessed until 3 s after
 * the 20 s period ends; G303-I: at most 8 fuel preloaded.
 *
 * <p>Paced in real time because Phoenix simulation runs on the wall-clock HAL timer.
 */
class AutoScoringHarness {
  static final int LEGAL_PRELOAD = 8;
  private static final double DT = 0.02;
  private static final double AUTO_SECONDS = 20.0;
  private static final double ASSESSMENT_SECONDS = 3.0;

  /**
   * The first volley alone empties a full hopper (the sim intake holds 40), so anything below 40
   * means the first cycle broke. Measured totals over 5 runs: 50, 58, 62, 66, 68; the spread is
   * maple-sim's unseeded hub outlet, which decides where scored fuel lands for the second sweep.
   */
  private static final int MIN_LEFT_TRENCH_FUEL = 40;

  /**
   * How far the robot may stray from the (alliance-flipped) path while the auto runs. This catches
   * a wrong flip, not tracking quality ({@code ChoreoTrackingHarness} and {@code
   * TrenchClearanceTest} own that). Measured on Left_Trench: the wrong flip type (mirrored instead
   * of rotated) puts the robot up to 4.87 m off, and no flip at all puts it 7.66 m off. Honest runs
   * peak at 0.31-0.41 m, on the fast turn near the end of the second sweep in open field (t = 14.4
   * s). 1.0 m sits well clear of both.
   */
  static final double MAX_PATH_DEVIATION_METERS = 1.0;

  @Test
  void leftTrenchScores() {
    Result result = run("LeftTrench", Alliance.Blue, Autos::leftTrench, "Left_Trench");
    assertTrue(
        result.scored() >= MIN_LEFT_TRENCH_FUEL,
        "Left Trench auto scored "
            + result.scored()
            + " fuel; expected at least "
            + MIN_LEFT_TRENCH_FUEL);
    assertTrue(
        result.maxPathDeviationMeters() <= MAX_PATH_DEVIATION_METERS,
        String.format("Robot strayed %.2f m from the path", result.maxPathDeviationMeters()));
  }

  /**
   * @param scored fuel in our own hub (fuel in the other hub scores nothing for us)
   * @param maxPathDeviationMeters furthest the true robot got from the path while the auto ran
   */
  record Result(int scored, double maxPathDeviationMeters) {}

  /**
   * @param trajectory Choreo trajectory the auto follows, flipped for Red the same way ChoreoLib
   *     flips it, to measure path deviation; null to skip that measurement.
   */
  static Result run(
      String label, Alliance alliance, Function<Autos, Command> autoFactory, String trajectory) {
    return run(label, alliance, autoFactory, trajectory, false);
  }

  /**
   * @param stockBothDepots turn off maple-sim's efficiency mode, which stocks only the depot at the
   *     far end of the field from the robot's alliance (maple-sim 0.4.0-beta has the depots
   *     swapped) and only every third row of neutral-zone fuel. Needed for any auto that uses its
   *     own depot.
   */
  static Result run(
      String label,
      Alliance alliance,
      Function<Autos, Command> autoFactory,
      String trajectory,
      boolean stockBothDepots) {
    List<Translation2d> path = new ArrayList<>();
    if (trajectory != null) {
      Trajectory<SwerveSample> traj = Choreo.<SwerveSample>loadTrajectory(trajectory).orElseThrow();
      for (SwerveSample sample : traj.samples()) {
        Pose2d pose = sample.getPose();
        path.add(
            (alliance == Alliance.Red ? ChoreoAllianceFlipUtil.flip(pose) : pose).getTranslation());
      }
    }
    double maxPathDeviation = 0.0;
    double maxPathDeviationTime = 0.0;

    HAL.initialize(500, 0);
    DriverStationSim.setAllianceStationId(
        alliance == Alliance.Red ? AllianceStationID.Red1 : AllianceStationID.Blue1);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();

    Arena2026Rebuilt arena = new Arena2026Rebuilt(false);
    arena.setEfficiencyMode(!stockBothDepots);
    SimulatedArena.overrideInstance(arena);
    SwerveDriveSimulation driveSimulation =
        new SwerveDriveSimulation(
            DriveConstants.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
    SimulatedArena.getInstance().placeGamePiecesOnField();

    Drive drive =
        new Drive(
            new GyroIOSim(driveSimulation.getGyroSimulation()),
            new ModuleIOTalonFXSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
            new ModuleIOTalonFXSim(TunerConstants.FrontRight, driveSimulation.getModules()[1]),
            new ModuleIOTalonFXSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]),
            new ModuleIOTalonFXSim(TunerConstants.BackRight, driveSimulation.getModules()[3]));
    drive.setSimPoseConsumer(driveSimulation::setSimulationWorldPose);

    Intake intake = new Intake(new IntakeIOSim(driveSimulation));
    intake.addGamePieces(LEGAL_PRELOAD);
    Extension extension = new Extension(new ExtensionIOSim());
    Hood hood = new Hood(new HoodIOSim());
    Kicker kicker = new Kicker(new RollerIOSim(KickerConstants.CONFIG));
    Feeder feeder = new Feeder(new RollerIOSim(FeederConstants.CONFIG));
    Indexer indexer = new Indexer(new RollerIOSim(IndexerConstants.CONFIG));
    Shooter shooter =
        new Shooter(
            new ShooterIOSim(
                driveSimulation::getSimulatedDriveTrainPose,
                drive::getFieldRelativeChassisSpeeds,
                hood::getAngle,
                () ->
                    feeder.isRunningForward()
                        && kicker.isRunningForward()
                        && intake.consumeGamePiece()));

    Autos autos = new Autos(drive, intake, extension, hood, shooter, kicker, feeder, indexer);
    Command auto = autoFactory.apply(autos);
    CommandScheduler.getInstance().schedule(auto);

    int maxHeld = intake.getStoredGamePieces();
    int collected = 0;
    int lastHeld = maxHeld;
    int scoredAtBuzzer = 0;
    double firstScoreTime = -1;
    double autoDoneTime = -1;
    long nextTickNanos = System.nanoTime();
    int ticks = (int) Math.round((AUTO_SECONDS + ASSESSMENT_SECONDS) / DT);
    for (int i = 0; i < ticks; i++) {
      long wait;
      while ((wait = nextTickNanos - System.nanoTime()) > 0) {
        LockSupport.parkNanos(wait);
      }
      nextTickNanos += (long) (DT * 1e9);
      double t = i * DT;

      if (t >= AUTO_SECONDS && DriverStation.isEnabled()) {
        // Buzzer: the robot is disabled, fuel already in the air keeps counting for 3 s.
        scoredAtBuzzer = hubFuel(alliance);
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        DriverStation.refreshData();
      }
      Unmanaged.feedEnable(100);
      CommandScheduler.getInstance().run();
      SimulatedArena.getInstance().simulationPeriodic();

      int held = intake.getStoredGamePieces();
      if (held > lastHeld) {
        collected += held - lastHeld;
      }
      lastHeld = held;
      maxHeld = Math.max(maxHeld, held);
      if (firstScoreTime < 0 && hubFuel(alliance) > 0) {
        firstScoreTime = t;
      }
      if (autoDoneTime < 0 && !CommandScheduler.getInstance().isScheduled(auto)) {
        autoDoneTime = t;
      }
      // From the second tick: the first one is where the auto resets odometry to the start pose.
      if (!path.isEmpty() && i > 0 && t < AUTO_SECONDS) {
        Translation2d truth = driveSimulation.getSimulatedDriveTrainPose().getTranslation();
        double nearest = Double.MAX_VALUE;
        for (Translation2d point : path) {
          nearest = Math.min(nearest, truth.getDistance(point));
        }
        if (nearest > maxPathDeviation) {
          maxPathDeviation = nearest;
          maxPathDeviationTime = t;
        }
      }
      if (i % 50 == 0) {
        Pose2d p = driveSimulation.getSimulatedDriveTrainPose();
        System.out.printf(
            "[auto %s] t=%5.2f pose=(%.2f, %.2f, %4.0fdeg) held=%d hub=%d%n",
            label, t, p.getX(), p.getY(), p.getRotation().getDegrees(), held, hubFuel(alliance));
      }
    }

    int scored = hubFuel(alliance);
    System.out.printf(
        "[auto %s] SUMMARY scored=%d (at buzzer %d) preload=%d collected=%d maxHeld=%d"
            + " leftInRobot=%d firstScore=%.2fs autoCommandEnded=%.2fs maxPathDeviation=%.3fm (t=%.2fs)%n",
        label,
        scored,
        scoredAtBuzzer,
        LEGAL_PRELOAD,
        collected,
        maxHeld,
        intake.getStoredGamePieces(),
        firstScoreTime,
        autoDoneTime,
        maxPathDeviation,
        maxPathDeviationTime);
    return new Result(scored, maxPathDeviation);
  }

  /** Fuel in this alliance's own hub; fuel in the other hub scores nothing for us. */
  private static int hubFuel(Alliance alliance) {
    SimulatedArena arena = SimulatedArena.getInstance();
    Double v =
        (alliance == Alliance.Red ? arena.redScoringBreakdown : arena.blueScoringBreakdown)
            .get("TotalFuelInHub");
    return v == null ? 0 : (int) (double) v;
  }
}
