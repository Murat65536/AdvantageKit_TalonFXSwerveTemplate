package frc.robot.subsystems.extension;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

/**
 * Hopper extension: one NEO on a Spark MAX. Only a reverse (retracted) limit switch exists, so
 * extending runs open-loop on a timer and the encoder is re-zeroed each time the retract switch
 * trips.
 */
public class ExtensionConstants {
  // Motor config
  public static final int MOTOR_CAN_ID = 4;
  public static final Current MOTOR_CURRENT_LIMIT = Amps.of(50);

  // Commanded voltages (duty cycles from the previous robot code scaled to 12 V)
  public static final Voltage EXTEND_VOLTAGE = Volts.of(0.8 * 12.0);
  public static final Voltage RETRACT_VOLTAGE = Volts.of(-0.5 * 12.0);

  /** How long to drive outward before the extension is considered fully extended. */
  public static final Time EXTEND_TIME = Seconds.of(0.51);

  /**
   * While feeding fuel to the shooter the hopper is slowly pulled inward, ramping from the start to
   * the end voltage over {@link #CREEP_RAMP_TIME}, to keep fuel flowing into the indexer.
   */
  public static final Voltage CREEP_IN_START_VOLTAGE = Volts.of(-0.2 * 12.0);

  public static final Voltage CREEP_IN_END_VOLTAGE = Volts.of(-0.55 * 12.0);
  public static final Time CREEP_RAMP_TIME = Seconds.of(1.0);

  // Sim travel model (motor rotations; position is zeroed at the retract switch)
  public static final double MIN_POSITION_ROT = 0.0;
  public static final double SIM_RPS_PER_VOLT = 0.75;
  /** Sim only: travel reached after EXTEND_TIME at EXTEND_VOLTAGE. */
  public static final double SIM_MAX_POSITION_ROT =
      EXTEND_VOLTAGE.in(Volts) * SIM_RPS_PER_VOLT * EXTEND_TIME.in(Seconds);

  // AdvantageScope component pose model (robot-relative)
  public static final Translation3d COMPONENT_ZERO_TRANSLATION = new Translation3d(0, 0, 0);
  public static final double COMPONENT_METERS_PER_ROTATION = Units.inchesToMeters(4);
  public static final double COMPONENT_MAX_METERS =
      SIM_MAX_POSITION_ROT * COMPONENT_METERS_PER_ROTATION;
  public static final double COMPONENT_SLOPE_DEG = 14.0;
}
