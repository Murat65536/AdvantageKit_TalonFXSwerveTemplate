package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;

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

  /** Hood angle = MAX_HOOD_ANGLE - encoderPosition. */
  public static Angle encoderPositionToAngle(double encoderPositionDeg) {
    return MAX_HOOD_ANGLE.minus(Degrees.of(encoderPositionDeg));
  }

  public static final Translation3d HOOD_PIVOT_TRANSLATION = new Translation3d(0.0, 0.0, 0.0);

  // Closed-loop tuning (0.05 duty cycle per degree from the previous robot code, scaled to 12 V)
  public static final double HOOD_KP_VOLTS_PER_DEG = 0.05 * 12.0;
  public static final Voltage MAX_CONTROL_VOLTAGE = Volts.of(8.0);

  /** Per-press manual trim applied on top of the calculated launch angle. */
  public static final Angle TRIM_STEP = Degrees.of(0.5);

  public static final Angle MAX_TRIM = Degrees.of(5.0);

  // Sim tuning
  public static final double HOOD_SIM_RESPONSE_PER_SECOND = 9.0;
  public static final AngularVelocity HOOD_SIM_MAX_VELOCITY = DegreesPerSecond.of(160.0);
  public static final double HOOD_SIM_KP_VOLTS_PER_DEG = 0.14;
}
