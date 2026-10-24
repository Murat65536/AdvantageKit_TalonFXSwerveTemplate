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
  // MIN/MAX are the physical hard stops and set the encoder geometry. Never command MIN: driving
  // the hood to the shallow stop breaks it. Targets are clamped to HOOD_SOFT_MIN_ANGLE instead.
  public static final Angle MIN_HOOD_ANGLE = Degrees.of(50); // no retraction (physical stop)
  public static final Angle MAX_HOOD_ANGLE = Degrees.of(76); // full retraction
  public static final Angle HOOD_ANGLE_TOLERANCE = Degrees.of(0.75);

  /**
   * Shallowest angle the hood is ever sent to: shots, trim, the systems check and (once zeroed) the
   * Pit-tab jog. Going all the way to the 50 deg stop breaks the hood mechanically. Shot-table
   * entries shallower than this (240 in at 53.5, 250 in at 54.5) are clamped to it.
   */
  public static final Angle HOOD_SOFT_MIN_ANGLE = Degrees.of(55);

  /** Clamp a requested hood angle to what may be commanded: soft minimum to the steep stop. */
  public static double clampTargetDeg(double angleDeg) {
    return MathUtil.clamp(angleDeg, HOOD_SOFT_MIN_ANGLE.in(Degrees), MAX_HOOD_ANGLE.in(Degrees));
  }

  /**
   * Whether a jog at {@code jogVolts} (positive is shallower) may keep driving at {@code angleDeg}.
   * A zeroed hood stops jogging shallower at the soft minimum. An unzeroed hood's angle can't be
   * trusted, so the jog is left to the person watching it.
   */
  public static boolean jogAllowed(double jogVolts, double angleDeg, boolean zeroed) {
    return !(zeroed && jogVolts > 0.0 && angleDeg <= HOOD_SOFT_MIN_ANGLE.in(Degrees));
  }

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

  /** Hood angle from the Spark's reported absolute position, which already has its zero applied. */
  public static Angle reportedEncoderToAngle(double reportedEncoderDeg) {
    return encoderPositionToAngle(unwrapEncoderPosition(reportedEncoderDeg));
  }

  /**
   * After zeroing, the Spark must report within this of 0 (the steep stop) or the zero is rejected.
   */
  public static final double ZERO_VERIFY_TOLERANCE_DEG = 1.0;

  /**
   * Whether a reported encoder position is the steep stop: 0, allowing for the wrap at a period.
   */
  public static boolean readsAsSteepStop(double reportedEncoderDeg) {
    double wrapped =
        MathUtil.inputModulus(
            reportedEncoderDeg,
            -ENCODER_POSITION_CONVERSION_FACTOR / 2.0,
            ENCODER_POSITION_CONVERSION_FACTOR / 2.0);
    return Math.abs(wrapped) <= ZERO_VERIFY_TOLERANCE_DEG;
  }

  /**
   * Spark zero offsets (rotations, [0, 1)) that make the current position report as 0, given what
   * it reports now and the offset it currently has. This is the same setting REV Hardware Client
   * writes, and it lives in the Spark's flash.
   *
   * <p>REV specifies the offset "as if the zero offset was 0, the conversion factor 1, and inverted
   * false", which says it is subtracted from the raw reading before inversion. That is candidate 0.
   * In case the firmware applies it after inversion instead, candidate 1 covers that order. The two
   * are equal when the encoder is not inverted. The caller tries them in order and keeps the first
   * that the Spark confirms by reporting 0.
   */
  public static double[] sparkZeroOffsetCandidates(
      double reportedEncoderDeg, double currentOffsetRot, boolean inverted) {
    double reportedRot = wrapRotations(reportedEncoderDeg / ENCODER_POSITION_CONVERSION_FACTOR);
    // Offset before inversion: reported = inv(raw - offset), so raw = inv(reported) + offset.
    double beforeInversion =
        wrapRotations((inverted ? 1.0 - reportedRot : reportedRot) + currentOffsetRot);
    // Offset after inversion: reported = inv(raw) - offset, so the new offset is inv(raw).
    double afterInversion = wrapRotations(reportedRot + currentOffsetRot);
    return new double[] {beforeInversion, afterInversion};
  }

  private static double wrapRotations(double rotations) {
    double wrapped = rotations % 1.0;
    return wrapped < 0.0 ? wrapped + 1.0 : wrapped;
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

  /**
   * Pit-tab jog for zeroing: each press drives the hood open loop at this voltage for {@link
   * #HOOD_JOG_SECONDS}. Negative volts move it steeper. NOT MEASURED: if a press does not move the
   * hood, raise the voltage a little; if one press moves it more than a few degrees, lower it.
   */
  public static final Voltage HOOD_JOG_VOLTAGE = Volts.of(1.5);

  public static final double HOOD_JOG_SECONDS = 0.2;

  /** Per-press manual trim applied on top of the calculated launch angle. */
  public static final Angle TRIM_STEP = Degrees.of(0.5);

  public static final Angle MAX_TRIM = Degrees.of(5.0);

  // Sim tuning
  public static final double HOOD_SIM_RESPONSE_PER_SECOND = 9.0;
  public static final AngularVelocity HOOD_SIM_MAX_VELOCITY = DegreesPerSecond.of(160.0);
  public static final double HOOD_SIM_KP_VOLTS_PER_DEG = 0.14;
}
