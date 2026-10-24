package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

/**
 * Manual mode, ported from the 2026RobotProject: a dashboard toggle that swaps the automatic shot
 * for an operator-set flywheel speed and hood angle. Values, steps and limits match that project.
 * The hood is still kept off its 50 deg hard stop by the soft minimum (see {@link
 * HoodConstants#clampTargetDeg}).
 */
public class ManualShot {
  public static final double DEFAULT_RPM = 2900.0;
  public static final double RPM_STEP = 50.0;
  public static final double MIN_RPM = 2000.0;
  /** Vortex free speed (6784 RPM) rounded down to a whole step, as in the old project. */
  public static final double MAX_RPM = Math.floor(6784.0 / RPM_STEP) * RPM_STEP;

  public static final double DEFAULT_HOOD_DEG = 70.0;
  public static final double HOOD_STEP_DEG = 2.0;

  public static final String TOGGLE_KEY = "/SmartDashboard/Match/Manual Mode";

  private final BooleanSupplier enabled;
  private double rpm = DEFAULT_RPM;
  private double hoodDeg = DEFAULT_HOOD_DEG;

  /** Reads Manual Mode from the dashboard toggle; it starts off every boot. */
  public ManualShot() {
    this(new LoggedNetworkBoolean(TOGGLE_KEY, false));
  }

  /** For tests: Manual Mode comes from {@code enabled}. */
  public ManualShot(BooleanSupplier enabled) {
    this.enabled = enabled;
  }

  public boolean isEnabled() {
    return enabled.getAsBoolean();
  }

  public double getRpm() {
    return rpm;
  }

  public double getHoodDeg() {
    return hoodDeg;
  }

  /**
   * Nudges the manual flywheel speed by {@code deltaRpm}, within {@link #MIN_RPM}..{@link
   * #MAX_RPM}.
   */
  public Command adjustRpm(double deltaRpm) {
    return Commands.runOnce(() -> rpm = MathUtil.clamp(rpm + deltaRpm, MIN_RPM, MAX_RPM))
        .ignoringDisable(true)
        .withName("ManualShotAdjustRpm");
  }

  /** Nudges the manual hood angle by {@code deltaDeg}, within the hood's allowed range. */
  public Command adjustHood(double deltaDeg) {
    return Commands.runOnce(() -> hoodDeg = HoodConstants.clampTargetDeg(hoodDeg + deltaDeg))
        .ignoringDisable(true)
        .withName("ManualShotAdjustHood");
  }

  /** Puts the manual speed and angle back to their defaults. */
  public Command reset() {
    return Commands.runOnce(
            () -> {
              rpm = DEFAULT_RPM;
              hoodDeg = DEFAULT_HOOD_DEG;
            })
        .ignoringDisable(true)
        .withName("ManualShotReset");
  }

  /** Spin the flywheel at the manual speed, with the kicker running, until interrupted. */
  public Command spinShooter(Shooter shooter, Kicker kicker) {
    return Commands.runEnd(() -> shooter.setVelocity(RPM.of(rpm)), shooter::stop, shooter)
        .alongWith(kicker.shoot())
        .withName("ManualShotSpin");
  }

  /** Hold the hood at the manual angle (the driver's trim is ignored) until interrupted. */
  public Command holdHood(Hood hood) {
    return Commands.run(() -> hood.setTargetAngleUntrimmed(Degrees.of(hoodDeg)), hood)
        .withName("ManualShotHood");
  }
}
