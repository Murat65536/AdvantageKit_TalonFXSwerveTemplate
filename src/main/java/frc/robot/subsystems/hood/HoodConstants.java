package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class HoodConstants {
  // Motor and encoder config
  public static final int LEFT_MOTOR_CAN_ID = 20;
  public static final int RIGHT_MOTOR_CAN_ID = 21;
  public static final Current MOTOR_CURRENT_LIMIT = Amps.of(20);

  // Geometry and limits
  public static final Angle MIN_HOOD_ANGLE = Degrees.of(54);
  public static final Angle MAX_HOOD_ANGLE = Degrees.of(73);
  public static final Angle HOOD_ABSOLUTE_ENCODER_ZERO_OFFSET = Degrees.of(0);
  public static final Angle HOOD_ANGLE_TOLERANCE = Degrees.of(0.75);

  // Closed-loop tuning
  public static final double HOOD_KP_VOLTS_PER_DEG = 0.20;
  public static final Voltage MAX_CONTROL_VOLTAGE = Volts.of(8.0);

  // Sim tuning
  public static final double HOOD_SIM_RESPONSE_PER_SECOND = 9.0;
  public static final AngularVelocity HOOD_SIM_MAX_VELOCITY = DegreesPerSecond.of(160.0);
  public static final double HOOD_SIM_KP_VOLTS_PER_DEG = 0.14;
}
