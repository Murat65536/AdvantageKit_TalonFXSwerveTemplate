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
  public static final Time EXTEND_TIME = Seconds.of(0.825);

  /**
   * Safety cap on a retract, in case the reverse limit switch fails or comes unplugged: 4x
   * EXTEND_TIME, the same limit at which the systems check fails a retract as too slow. Scales with
   * EXTEND_TIME because a longer extend means more travel to pull back in.
   */
  public static final Time RETRACT_TIMEOUT = EXTEND_TIME.times(4.0);

  /**
   * While feeding fuel to the shooter the hopper is pulled inward to keep fuel flowing into the
   * indexer: nothing for the Retract Delay (Match tab, {@link #RETRACT_DELAY_KEY}), then a ramp
   * from the start to the end voltage over {@link #CREEP_RAMP_TIME}, then the end voltage until the
   * retract switch trips.
   *
   * <p>The voltages are the 2026RobotProject's: 25% duty rising to 40%. The previous Wescavator
   * creep (2.4 to 6.6 V) pulled the hopper in too quickly. That project also waited 0.5 s before
   * pulling in; here the wait is the dashboard's Retract Delay, which starts at 0 s every boot.
   */
  public static final String RETRACT_DELAY_KEY = "/SmartDashboard/Match/Retract Delay";

  public static final double DEFAULT_RETRACT_DELAY_SECONDS = 0.0;

  public static final Voltage CREEP_IN_START_VOLTAGE = Volts.of(-0.25 * 12.0);
  public static final Voltage CREEP_IN_END_VOLTAGE = Volts.of(-0.4 * 12.0);
  public static final Time CREEP_RAMP_TIME = Seconds.of(0.5);

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
