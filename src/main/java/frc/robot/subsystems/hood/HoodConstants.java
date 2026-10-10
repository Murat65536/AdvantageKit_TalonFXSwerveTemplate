package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class HoodConstants {
  // Motor and encoder config (two NEO 550s on Spark MAX, absolute encoder on the left Spark)
  public static final int LEFT_MOTOR_CAN_ID = 13;
  public static final int RIGHT_MOTOR_CAN_ID = 12;
  public static final Current MOTOR_CURRENT_LIMIT = Amps.of(20);

  // Geometry and limits. Launch angle is measured from horizontal; a fully retracted hood gives
  // the highest (steepest) launch angle.
  public static final Angle MIN_HOOD_ANGLE = Degrees.of(50); // no retraction
  public static final Angle MAX_HOOD_ANGLE = Degrees.of(76); // full retraction
  public static final Angle HOOD_ANGLE_TOLERANCE = Degrees.of(0.75);

  /**
   * The absolute encoder is inverted and geared so one encoder revolution spans twice the hood's
   * travel. Encoder position (after conversion) is in "hood degrees" and wraps at this value.
   */
  public static final boolean ABSOLUTE_ENCODER_INVERTED = true;

  public static final double ENCODER_POSITION_CONVERSION_FACTOR =
      (MAX_HOOD_ANGLE.in(Degrees) - MIN_HOOD_ANGLE.in(Degrees)) * 2.0;

  /**
   * Maps the raw encoder reading (0 to one period) onto a continuous range around the hood's
   * travel. The hood only covers half the encoder's period, so the wrap point goes in the middle of
   * the unreachable half, not at the fully retracted end. Readings up to half the travel past
   * either hard stop stay continuous, and the error between any two hood angles never wraps.
   */
  public static double unwrapEncoderPosition(double rawEncoderDeg) {
    double travelMidDeg = (MAX_HOOD_ANGLE.in(Degrees) - MIN_HOOD_ANGLE.in(Degrees)) / 2.0;
    return MathUtil.inputModulus(
        rawEncoderDeg,
        travelMidDeg - ENCODER_POSITION_CONVERSION_FACTOR / 2.0,
        travelMidDeg + ENCODER_POSITION_CONVERSION_FACTOR / 2.0);
  }

  /** Hood angle = MAX_HOOD_ANGLE - encoderPosition (unwrapped). */
  public static Angle encoderPositionToAngle(double encoderPositionDeg) {
    return MAX_HOOD_ANGLE.minus(Degrees.of(encoderPositionDeg));
  }

  public static final Translation3d HOOD_PIVOT_TRANSLATION = new Translation3d(0.0, 0.0, 0.0);

  // Closed-loop tuning (0.05 duty cycle per degree from the previous robot code, scaled to 12 V)
  public static final double HOOD_KP_VOLTS_PER_DEG = 0.05 * 12.0;
  public static final Voltage MAX_CONTROL_VOLTAGE = Volts.of(8.0);

  /**
   * Motion limits for the profiled setpoint (real robot). Full-step moves at 8 V slammed the hood
   * into its hard stop and skipped the motor gears. At these limits a full 26 deg move takes about
   * 0.25 s. NOT MEASURED: raise them if the hood stays smooth, lower them if it still skips.
   */
  public static final double HOOD_MAX_VELOCITY_DEG_PER_SEC = 150.0;

  public static final double HOOD_MAX_ACCEL_DEG_PER_SEC_SQ = 1500.0;

  /**
   * Stall cutoff (real robot): both motors' current above this while the hood moves slower than
   * {@link #HOOD_STALL_VELOCITY_DEG_PER_SEC} for {@link #HOOD_STALL_TIME_SECONDS} means it is
   * pushing into something. Output is cut until the robot is disabled.
   */
  public static final Current HOOD_STALL_CURRENT = Amps.of(25);

  public static final double HOOD_STALL_VELOCITY_DEG_PER_SEC = 5.0;
  public static final double HOOD_STALL_TIME_SECONDS = 0.3;

  /** Per-press manual trim applied on top of the calculated launch angle. */
  public static final Angle TRIM_STEP = Degrees.of(0.5);

  public static final Angle MAX_TRIM = Degrees.of(5.0);

  // Sim tuning
  public static final double HOOD_SIM_RESPONSE_PER_SECOND = 9.0;
  public static final AngularVelocity HOOD_SIM_MAX_VELOCITY = DegreesPerSecond.of(160.0);
  public static final double HOOD_SIM_KP_VOLTS_PER_DEG = 0.14;
}
