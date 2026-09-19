package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

public class ShooterConstants {
  // Motor config: four Neo Vortex on Spark MAX. Left-top leads.
  public static final int LEFT_TOP_MOTOR_CAN_ID = 5;
  public static final int LEFT_BOTTOM_MOTOR_CAN_ID = 3;
  public static final int RIGHT_TOP_MOTOR_CAN_ID = 16;
  public static final int RIGHT_BOTTOM_MOTOR_CAN_ID = 15;
  public static final boolean LEADER_INVERTED = true;
  public static final Current CURRENT_LIMIT = Amps.of(70);

  // Onboard velocity loop (duty cycle per RPM)
  public static final double VELOCITY_KP = 0.0002;
  public static final double VELOCITY_KI = 0.0;
  public static final double VELOCITY_KD = 0.0;
  public static final double VELOCITY_KS = 0.0;
  public static final double VELOCITY_KV = 0.00187;
  public static final double VELOCITY_KA = 0.0;

  // Voltages
  public static final Voltage SHOOT_VOLTAGE = Volts.of(12);
  public static final Voltage REVERSE_VOLTAGE = Volts.of(-4);

  // Sim constants
  public static final double FLYWHEEL_GEAR_RATIO = 1.0; // direct drive
  public static final double FLYWHEEL_MOI = 0.004; // kg*m^2

  // Projectile constants
  public static final Translation3d BALL_EXIT_TRANSLATION =
      new Translation3d(Units.inchesToMeters(6.384), 0, Units.inchesToMeters(21.2315));
  public static final Distance FLYWHEEL_EFFECTIVE_RADIUS = Inches.of(2.0);
  public static final double EXIT_VELOCITY_SLIP_FACTOR = 0.58;
  public static final double HUB_FLYWHEEL_RPM_SCALE = 0.80;
  public static final AngularVelocity SHOOTER_READY_VELOCITY = RPM.of(2500);
  public static final double EXIT_VELOCITY_PER_FLYWHEEL_RAD_PER_SEC =
      FLYWHEEL_EFFECTIVE_RADIUS.in(Meters) * EXIT_VELOCITY_SLIP_FACTOR;
  public static final LinearVelocity SHOOTER_EXIT_VELOCITY =
      MetersPerSecond.of(
          SHOOTER_READY_VELOCITY.in(RadiansPerSecond) * EXIT_VELOCITY_PER_FLYWHEEL_RAD_PER_SEC);
  public static final Time SHOOTER_SHOT_PERIOD = Seconds.of(0.03);

  /**
   * Robot-relative X offset of the shooter barrel used for range calculation. NOTE: the previous
   * robot code used -5.202363 in (barrel behind robot center, matching a rear-facing shooter); the
   * sign here is kept as tuned with the current RPM map. Verify on the robot.
   */
  public static final double SHOOTER_OFFSET_X_METERS = Units.inchesToMeters(5.202363);

  // Dynamic shot limits and conversion
  public static final LinearVelocity MIN_DYNAMIC_EXIT_VELOCITY = MetersPerSecond.of(8.0);
  public static final LinearVelocity MAX_DYNAMIC_EXIT_VELOCITY = MetersPerSecond.of(22.0);
  public static final AngularVelocity MAX_FLYWHEEL_VELOCITY = RPM.of(6784.0);

  // Ready / feed gating
  public static final AngularVelocity SHOOTER_AT_SPEED_TOLERANCE = RPM.of(150.0);
  /** Heading tolerance for the drive to be considered aimed at the target. */
  public static final double AIM_TOLERANCE_RAD = Math.toRadians(5.0);
  /**
   * Once feeding has started, the ready condition must be false for this long before feeding stops.
   * Prevents the flywheel speed dip from each shot from chattering the feed path.
   */
  public static final Time FEED_STOP_DEBOUNCE = Seconds.of(0.25);
  /** How long the intake roller runs at the start of each feed to push loose fuel inward. */
  public static final Time FEED_INTAKE_PULSE = Seconds.of(0.5);

  // Sim-only velocity loop
  public static final double SHOOTER_SIM_VELOCITY_KP_VOLTS_PER_RAD_PER_SEC = 0.03;
}
