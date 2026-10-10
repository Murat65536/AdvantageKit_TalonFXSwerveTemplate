package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.HubShiftUtil;
import java.util.function.DoubleConsumer;
import org.littletonrobotics.junction.Logger;

/**
 * The only code that writes the driver controller's rumble.
 *
 * <p>Each feature sets its own level, and the controller always gets the strongest one. Features
 * never write the controller directly: when two did, one ending turned off the other's rumble while
 * it was still active (the hub warning ending cut the aiming rumble mid-shot).
 *
 * <p>None of these commands require a subsystem, so they run alongside whatever the driver is
 * doing.
 */
public class DriverRumble {
  /** Constant rumble while the one-button shot is aiming. */
  public static final double AIMING_LEVEL = 0.8;

  /** How long before our hub activates the warning ramp starts. */
  public static final double HUB_WARNING_SECONDS = 3.0;

  private final DoubleConsumer controller;
  private double aimingLevel = 0.0;
  private double hubWarningLevel = 0.0;

  /**
   * @param controller writes a 0-1 rumble level to the physical controller
   */
  public DriverRumble(DoubleConsumer controller) {
    this.controller = controller;
  }

  /** Rumbles at {@link #AIMING_LEVEL} while scheduled. */
  public Command whileAiming() {
    return Commands.startEnd(() -> setAiming(AIMING_LEVEL), () -> setAiming(0.0))
        .withName("RumbleWhileAiming");
  }

  /**
   * Logs the hub shift state and ramps the rumble up over the last {@link #HUB_WARNING_SECONDS}
   * before our hub becomes active, so the driver can line up a shot early. Only gives feedback; it
   * never blocks shooting.
   */
  public Command hubWarning() {
    return Commands.run(
            () -> {
              double matchTime = DriverStation.getMatchTime();
              logHubState(matchTime);
              setHubWarning(hubWarningLevel(matchTime));
            })
        .finallyDo(() -> setHubWarning(0.0))
        .withName("HubActivationWarning");
  }

  /**
   * Hub warning rumble for a given teleop match time remaining: 0 until {@link
   * #HUB_WARNING_SECONDS} before our hub activates, then a linear ramp to 1.
   */
  public static double hubWarningLevel(double matchTime) {
    if (HubShiftUtil.isHubActive(matchTime)) {
      return 0.0;
    }
    double secondsUntilActive = HubShiftUtil.secondsUntilHubActive(matchTime);
    if (secondsUntilActive > HUB_WARNING_SECONDS) {
      return 0.0;
    }
    return MathUtil.clamp(1.0 - secondsUntilActive / HUB_WARNING_SECONDS, 0.0, 1.0);
  }

  private void setAiming(double level) {
    aimingLevel = level;
    apply();
  }

  private void setHubWarning(double level) {
    hubWarningLevel = level;
    apply();
  }

  private void apply() {
    double level = Math.max(aimingLevel, hubWarningLevel);
    Logger.recordOutput("DriverRumble/Aiming", aimingLevel);
    Logger.recordOutput("DriverRumble/HubWarning", hubWarningLevel);
    Logger.recordOutput("DriverRumble/Output", level);
    controller.accept(level);
  }

  private static void logHubState(double matchTime) {
    boolean hubActive = HubShiftUtil.isHubActive(matchTime);
    double secondsUntilActive = HubShiftUtil.secondsUntilHubActive(matchTime);
    Logger.recordOutput("Hub/Shift", HubShiftUtil.getShift(matchTime).toString());
    Logger.recordOutput("Hub/Active", hubActive);
    Logger.recordOutput("Hub/SecondsUntilActive", secondsUntilActive);
    Logger.recordOutput(
        "Hub/ActivatingSoon", !hubActive && secondsUntilActive <= HUB_WARNING_SECONDS);
    Logger.recordOutput(
        "Hub/InactiveFirstAlliance",
        HubShiftUtil.getInactiveFirstAlliance().map(Enum::toString).orElse("unknown"));
  }
}
