package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

public class ShooterConstants {
  // Motor config
  public static final int TOP_LEFT_MOTOR_CAN_ID = 43;
  public static final int TOP_RIGHT_MOTOR_CAN_ID = 45;
  public static final int BOTTOM_LEFT_MOTOR_CAN_ID = 44;
  public static final int BOTTOM_RIGHT_MOTOR_CAN_ID = 46;
  public static final Current CURRENT_LIMIT = Amps.of(40);

  // Voltages
  public static final Voltage SHOOT_VOLTAGE = Volts.of(12);
  public static final Voltage REVERSE_VOLTAGE = Volts.of(-4);

  // Sim constants
  public static final double FLYWHEEL_GEAR_RATIO = 1.0; // direct drive
  public static final double FLYWHEEL_MOI = 0.004; // kg*m^2

  // Hub geometry constants (2022 style)
  public static final double HUB_EDGE_DISTANCE_FROM_DRIVER_STATION = Units.inchesToMeters(158.6);
  public static final double HUB_LENGTH = Units.inchesToMeters(47.0);
  public static final Translation2d HUB_TRANSLATION =
      new Translation2d(
          HUB_EDGE_DISTANCE_FROM_DRIVER_STATION + HUB_LENGTH / 2.0, FlippingUtil.fieldSizeY / 2.0);
  public static final Distance HUB_HEIGHT = Inches.of(104.0);

  // Projectile constants
  public static final Distance SHOOTER_HEIGHT = Inches.of(28);
  public static final Distance FLYWHEEL_EFFECTIVE_RADIUS = Inches.of(2.0);
  public static final double EXIT_VELOCITY_SLIP_FACTOR = 0.58;
  public static final double HUB_FLYWHEEL_RPM_SCALE = 0.80;
  public static final Angle SHOOTER_ANGLE = Degrees.of(38);
  public static final Angle MIN_DYNAMIC_SHOOTER_ANGLE = Degrees.of(34.0);
  public static final Angle MAX_DYNAMIC_SHOOTER_ANGLE = Degrees.of(58.0);
  public static final Angle DYNAMIC_SHOOTER_ANGLE_STEP = Degrees.of(1.0);
  public static final AngularVelocity SHOOTER_READY_VELOCITY = RPM.of(2500);
  public static final double EXIT_VELOCITY_PER_FLYWHEEL_RAD_PER_SEC =
      FLYWHEEL_EFFECTIVE_RADIUS.in(Meters) * EXIT_VELOCITY_SLIP_FACTOR;
  public static final LinearVelocity SHOOTER_EXIT_VELOCITY =
      MetersPerSecond.of(
          SHOOTER_READY_VELOCITY.in(RadiansPerSecond) * EXIT_VELOCITY_PER_FLYWHEEL_RAD_PER_SEC);
  public static final Time SHOOTER_SHOT_PERIOD = Seconds.of(0.18);

  public static final double SHOOTER_OFFSET_X_METERS = Units.inchesToMeters(5.202363);

  // Dynamic shot limits and conversion
  public static final LinearVelocity MIN_DYNAMIC_EXIT_VELOCITY = MetersPerSecond.of(8.0);
  public static final LinearVelocity MAX_DYNAMIC_EXIT_VELOCITY = MetersPerSecond.of(22.0);
  public static final AngularVelocity MAX_FLYWHEEL_VELOCITY = RPM.of(6784.0);

  // Velocity control tuning
  public static final double SHOOTER_VELOCITY_KP_VOLTS_PER_RPM = 0.0025;
  public static final double SHOOTER_VELOCITY_FF_VOLTS_PER_RPM =
      12.0 / MAX_FLYWHEEL_VELOCITY.in(RPM);
  public static final double SHOOTER_SIM_VELOCITY_KP_VOLTS_PER_RAD_PER_SEC = 0.03;
  public static final AngularVelocity SHOOTER_AT_SPEED_TOLERANCE = RPM.of(150.0);
}
