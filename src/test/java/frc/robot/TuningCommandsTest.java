package frc.robot;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.TuningCommands;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSim;
import java.util.concurrent.locks.LockSupport;
import org.junit.jupiter.api.Test;

/**
 * Smoke test: TuningCommands.captureTrackingNoise runs end-to-end against the real sim IOs. Follows
 * the ChoreoTrackingHarness convention (real-time-paced ticks + DriverStationSim enabled), since
 * Command.withTimeout is driven by the wall-clock FPGA timestamp, not tick count.
 */
class TuningCommandsTest {
  private static final double DT = 0.02;

  @Test
  void captureCompletesAndLogsResult() {
    HAL.initialize(500, 0);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();

    Shooter shooter =
        new Shooter(
            new ShooterIOSim(
                () -> new edu.wpi.first.math.geometry.Pose2d(),
                () -> new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                () -> Degrees.of(59),
                () -> false));
    Hood hood = new Hood(new HoodIOSim());

    var command = TuningCommands.captureTrackingNoiseAtMidRangeShot(shooter, hood);
    CommandScheduler.getInstance().schedule(command);

    // 2s settle + 5s capture + margin, real-time paced (see class doc).
    double totalSeconds = 2.0 + 5.0 + 1.0;
    long nextTickNanos = System.nanoTime();
    for (double t = 0.0; t < totalSeconds; t += DT) {
      long nanosUntilTick;
      while ((nanosUntilTick = nextTickNanos - System.nanoTime()) > 0) {
        LockSupport.parkNanos(nanosUntilTick);
      }
      nextTickNanos += (long) (DT * 1e9);
      Unmanaged.feedEnable(100);
      CommandScheduler.getInstance().run();
    }

    assertTrue(
        !CommandScheduler.getInstance().isScheduled(command), "command should have finished");
  }
}
