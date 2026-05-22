package frc.robot.auton;

import choreo.trajectory.EventMarker;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.Drive;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/**
 * Follows a Choreo trajectory with a clock governor. Instead of advancing trajectory time at
 * wall-clock rate, time advances at {@code dt * f(error)}: f -> 1 when tracking is tight and f -> 0
 * when the robot falls behind, so the setpoint pauses and lets the robot catch up rather than
 * racing away.
 *
 * <p>This replaces ChoreoLib's {@code AutoFactory}/{@code AutoTrajectory} playback (which hardcodes
 * a wall-clock timeline with no governing hook) and re-implements event-marker firing on the
 * governed clock — so markers fire at the correct point on the path even if the robot was paused.
 */
public class GovernedTrajectoryCommand extends Command {
  /**
   * Below this translation error (m) the governor is inert (f = 1). Set above the normal transient
   * tracking-error peak (~0.37 m measured) so good autos run ungoverned.
   */
  private static final double ERR_LOW = 0.45;

  /** At or above this translation error (m) the trajectory is fully paused (f = 0). */
  private static final double ERR_HIGH = 0.85;

  /** If governed time fails to advance for this long (s), the robot is stuck — end the command. */
  private static final double STUCK_TIMEOUT = 3.0;

  private final Trajectory<SwerveSample> trajectory;
  private final Drive drive;
  private final Map<String, Command> eventCommands;
  private final List<EventMarker> markers;
  private final boolean[] markerFired;

  private final Timer wallTimer = new Timer();
  private double governedTime;
  private double lastWallTime;
  private double lastProgressWallTime;
  private boolean mirror;

  public GovernedTrajectoryCommand(
      Trajectory<SwerveSample> trajectory, Drive drive, Map<String, Command> eventCommands) {
    this.trajectory = trajectory;
    this.drive = drive;
    this.eventCommands = eventCommands;
    this.markers = trajectory.events();
    this.markerFired = new boolean[markers.size()];
    addRequirements(drive);
    setName("Auto_" + trajectory.name());
  }

  @Override
  public void initialize() {
    // Alliance is known at run time, not construction time — resolve the mirror flag here.
    mirror = DriverStation.getAlliance().map(a -> a == Alliance.Red).orElse(false);
    governedTime = 0.0;
    Arrays.fill(markerFired, false);
    trajectory.getInitialPose(mirror).ifPresent(drive::setPose);
    drive.resetChoreoControllers();
    wallTimer.restart();
    lastWallTime = 0.0;
    lastProgressWallTime = 0.0;
  }

  @Override
  public void execute() {
    double now = wallTimer.get();
    double dt = now - lastWallTime;
    lastWallTime = now;

    SwerveSample sample = trajectory.sampleAt(governedTime, mirror).orElse(null);
    if (sample == null) {
      return;
    }

    double error = drive.getPose().getTranslation().getDistance(sample.getPose().getTranslation());
    double f = MathUtil.clamp((ERR_HIGH - error) / (ERR_HIGH - ERR_LOW), 0.0, 1.0);

    drive.followChoreoSample(sample, f);

    // Fire any event markers the governed clock has now reached.
    for (int i = 0; i < markers.size(); i++) {
      EventMarker marker = markers.get(i);
      if (!markerFired[i] && marker.timestamp <= governedTime) {
        markerFired[i] = true;
        Command cmd = eventCommands.get(marker.event);
        if (cmd != null) {
          CommandScheduler.getInstance().schedule(cmd);
        }
      }
    }

    governedTime += dt * f;
    if (f > 0.0) {
      lastProgressWallTime = now;
    }

    Logger.recordOutput("Choreo/GovernorFactor", f);
    Logger.recordOutput("Choreo/GovernedTime", governedTime);
  }

  @Override
  public boolean isFinished() {
    return governedTime >= trajectory.getTotalTime()
        || wallTimer.get() - lastProgressWallTime > STUCK_TIMEOUT;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
