package frc.robot.subsystems.extension;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ExtensionConstants {
  // Motor config
  public static final int MOTOR_CAN_ID = 47;
  public static final Current MOTOR_CURRENT_LIMIT = Amps.of(40);

  // Commanded voltages
  public static final Voltage EXTEND_VOLTAGE = Volts.of(8.0);
  public static final Voltage RETRACT_VOLTAGE = Volts.of(-8.0);

  // Sim travel model
  public static final double MIN_POSITION_ROT = 0.0;
  public static final double MAX_POSITION_ROT = 4.0;
  public static final double SIM_RPS_PER_VOLT = 0.75;

  // AdvantageScope component pose model (robot-relative)
  public static final Translation3d COMPONENT_ZERO_TRANSLATION = new Translation3d(0, 0, 0);
  public static final double COMPONENT_METERS_PER_ROTATION = Units.inchesToMeters(4);
  public static final double COMPONENT_SLOPE_DEG = 14.0;
}
