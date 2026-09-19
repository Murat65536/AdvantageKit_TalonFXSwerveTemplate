package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.HubShiftUtil;
import java.util.function.DoubleConsumer;
import org.littletonrobotics.junction.Logger;

/**
 * Runs for all of teleop. Logs the hub shift state and ramps controller rumble during the final
 * seconds before our hub becomes active so the drivers can pre-stage a shot. Never blocks shooting.
 */
public class HubActivationWarningCommand extends Command {
  public static final double WARNING_SECONDS = 3.0;

  private final DoubleConsumer rumbleConsumer;
  private boolean wasWarning = false;

  public HubActivationWarningCommand(DoubleConsumer rumbleConsumer) {
    this.rumbleConsumer = rumbleConsumer;
  }

  @Override
  public void execute() {
    double matchTime = DriverStation.getMatchTime();
    boolean hubActive = HubShiftUtil.isHubActive(matchTime);
    double secondsUntilActive = HubShiftUtil.secondsUntilHubActive(matchTime);
    boolean warning = !hubActive && secondsUntilActive <= WARNING_SECONDS;

    Logger.recordOutput("Hub/Shift", HubShiftUtil.getShift(matchTime).toString());
    Logger.recordOutput("Hub/Active", hubActive);
    Logger.recordOutput("Hub/SecondsUntilActive", secondsUntilActive);
    Logger.recordOutput("Hub/ActivatingSoon", warning);
    Logger.recordOutput(
        "Hub/InactiveFirstAlliance",
        HubShiftUtil.getInactiveFirstAlliance().map(Enum::toString).orElse("unknown"));

    if (warning) {
      // Ramp from 0 to full rumble as activation approaches.
      double rumble = MathUtil.clamp(1.0 - secondsUntilActive / WARNING_SECONDS, 0.0, 1.0);
      rumbleConsumer.accept(rumble);
    } else if (wasWarning) {
      rumbleConsumer.accept(0.0);
    }
    wasWarning = warning;
  }

  @Override
  public void end(boolean interrupted) {
    rumbleConsumer.accept(0.0);
  }
}
