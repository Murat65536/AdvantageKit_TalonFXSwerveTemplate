package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.SystemsCheck;
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
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import java.util.concurrent.locks.LockSupport;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.junit.jupiter.api.Test;

/**
 * Runs the whole pit systems check, exactly as the dashboard button builds it, against the sim
 * robot. Every step must pass: this proves the caps are reachable by a healthy robot, so a real
 * FAIL means something is actually wrong. Paced in real time (Phoenix sim runs on the wall-clock
 * HAL timer); separate class because sim tests cannot share a JVM.
 */
class SystemsCheckSimHarness {
  private static final double DT = 0.02;

  @Test
  void healthySimRobotPassesEveryStep() {
    HAL.initialize(500, 0);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setFmsAttached(false);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();

    SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));
    SwerveDriveSimulation driveSimulation =
        new SwerveDriveSimulation(
            DriveConstants.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
    Drive drive =
        new Drive(
            new GyroIOSim(driveSimulation.getGyroSimulation()),
            new ModuleIOTalonFXSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
            new ModuleIOTalonFXSim(TunerConstants.FrontRight, driveSimulation.getModules()[1]),
            new ModuleIOTalonFXSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]),
            new ModuleIOTalonFXSim(TunerConstants.BackRight, driveSimulation.getModules()[3]));
    VisionIO camera =
        new VisionIO() {
          @Override
          public void updateInputs(VisionIOInputs inputs) {
            inputs.connected = true;
          }
        };
    Vision vision = new Vision((pose, time, stdDevs) -> {}, camera, camera);
    Intake intake = new Intake(new IntakeIOSim(driveSimulation));
    Hood hood = new Hood(new HoodIOSim());
    Kicker kicker = new Kicker(new RollerIOSim(KickerConstants.CONFIG));
    Feeder feeder = new Feeder(new RollerIOSim(FeederConstants.CONFIG));
    Indexer indexer = new Indexer(new RollerIOSim(IndexerConstants.CONFIG));
    Extension extension = new Extension(new ExtensionIOSim());
    Shooter shooter =
        new Shooter(
            new ShooterIOSim(
                driveSimulation::getSimulatedDriveTrainPose,
                drive::getFieldRelativeChassisSpeeds,
                hood::getAngle,
                () -> false));

    SystemsCheck check =
        SystemsCheck.standard(
            new SystemsCheck.MemoryStore(),
            drive,
            vision,
            intake,
            indexer,
            feeder,
            kicker,
            shooter,
            hood,
            extension);
    Command run = check.run();
    CommandScheduler.getInstance().schedule(run);

    long next = System.nanoTime();
    double elapsed = 0.0;
    while (run.isScheduled() && elapsed < 120.0) {
      long wait;
      while ((wait = next - System.nanoTime()) > 0) {
        LockSupport.parkNanos(wait);
      }
      next += (long) (DT * 1e9);
      CommandScheduler.getInstance().run();
      SimulatedArena.getInstance().simulationPeriodic();
      elapsed += DT;
    }

    StringBuilder report = new StringBuilder();
    check
        .results()
        .forEach(
            (name, r) ->
                report.append(String.format("%-30s %-7s %s%n", name, r.status(), r.detail())));
    System.out.printf("SystemsCheck finished in %.1f s%n%s", elapsed, report);
    assertTrue(!run.isScheduled(), "check did not finish in 120 s\n" + report);
    assertEquals(SystemsCheck.Status.PASS, check.overall(), report.toString());
  }
}
