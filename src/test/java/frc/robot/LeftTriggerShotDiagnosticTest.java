package frc.robot;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShootCommands;
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
import frc.robot.subsystems.shooter.ShooterMath;
import java.util.concurrent.locks.LockSupport;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.junit.jupiter.api.Test;

/**
 * Reproduces the reported "left trigger does nothing in sim" symptom against the real sim stack,
 * and reports WHICH of the ready-gate conditions is blocking the shot.
 */
class LeftTriggerShotDiagnosticTest {
  private static final double DT = 0.02;

  @Test
  void diagnoseWhyShotDoesNotFire() {
    HAL.initialize(500, 0);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();

    SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));
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
    intake.addGamePieces(10);
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

    System.out.println("[diag] stored fuel at start = " + intake.getStoredGamePieces());

    Command aim = DriveCommands.joystickDriveAimAtTarget(drive, () -> 0.0, () -> 0.0);
    Command shoot =
        ShootCommands.shootAtTarget(
            drive, shooter, hood, kicker, feeder, indexer, intake, extension);
    CommandScheduler.getInstance().schedule(aim);
    CommandScheduler.getInstance().schedule(shoot);

    long nextTickNanos = System.nanoTime();
    // Window must outlast the longest time of flight in the shot table (~1.6s) plus the
    // time spent aiming, or fuel is still airborne when the test stops stepping the arena.
    final double windowSeconds = 8.0;
    for (int i = 0; i < (int) (windowSeconds / DT); i++) {
      long wait;
      while ((wait = nextTickNanos - System.nanoTime()) > 0) {
        LockSupport.parkNanos(wait);
      }
      nextTickNanos += (long) (DT * 1e9);
      Unmanaged.feedEnable(100);
      CommandScheduler.getInstance().run();
      SimulatedArena.getInstance().simulationPeriodic();

      // The bare harness has no vision, so wheel odometry drifts several degrees away
      // from the true pose and the robot "aims" confidently at the wrong bearing. On the
      // real robot vision corrects this. Seed odometry from ground truth each tick so this
      // test measures the SHOT TABLE rather than harness odometry drift.
      drive.setPose(driveSimulation.getSimulatedDriveTrainPose());

      if (i % 50 == 0 || i == (int) (windowSeconds / DT) - 1) {
        Pose2d truth = driveSimulation.getSimulatedDriveTrainPose();
        Pose2d odom = drive.getPose();
        var sol = shooter.getShotSolution();
        System.out.printf(
            "[diag] t=%.2fs storedFuel=%d flywheel=%b hood=%b kicker=%b aimedOdom=%b solved=%b"
                + " aimErrOdomDeg=%.2f aimErrTruthDeg=%.2f | dist=%.3fm hoodCmd=%.1f hoodAct=%.1f"
                + " rpmCmd=%.0f rpmAct=%.0f inRange=%b%n",
            i * DT,
            intake.getStoredGamePieces(),
            shooter.atSetpoint(),
            hood.atSetpoint(),
            kicker.atSetpoint(),
            ShooterMath.isAimed(odom),
            shooter.hasShotSolution(),
            Math.toDegrees(ShooterMath.getAimErrorRad(odom)),
            Math.toDegrees(ShooterMath.getAimErrorRad(truth)),
            sol == null ? -1.0 : sol.distanceMeters(),
            sol == null ? -1.0 : sol.launchAngle().in(Degrees),
            hood.getAngle().in(Degrees),
            sol == null ? -1.0 : sol.flywheelVelocity().in(RPM),
            shooter.getVelocity().in(RPM),
            sol != null && sol.inRange());
      }
    }

    int fuelScored = readBlueFuelInHub();
    System.out.printf(
        "[diag] TotalFuelInHub (blue) = %d, blueScore = %d%n",
        fuelScored, SimulatedArena.getInstance().getScore(true));
    assertTrue(fuelScored > 0, "no fuel scored in the hub after 4s of aiming and shooting");
  }

  /** Reads maple-sim's own scoring breakdown so the assertion uses the sim's verdict, not ours. */
  private static int readBlueFuelInHub() {
    Double v = SimulatedArena.getInstance().blueScoringBreakdown.get("TotalFuelInHub");
    return v == null ? -1 : (int) (double) v;
  }
}
