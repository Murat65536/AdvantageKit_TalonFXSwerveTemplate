package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
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

  // MapleSim projectile constants
  public static final Translation2d TOP_SHOOTER_POSITION_ON_ROBOT = new Translation2d(0.32, 0.12);
  public static final Translation2d BOTTOM_SHOOTER_POSITION_ON_ROBOT =
      new Translation2d(0.32, -0.12);
  public static final Distance SHOOTER_HEIGHT = Inches.of(28);
  public static final LinearVelocity SHOOTER_EXIT_VELOCITY = MetersPerSecond.of(16.0);
  public static final Angle SHOOTER_ANGLE = Degrees.of(38);
  public static final AngularVelocity SHOOTER_READY_VELOCITY = RPM.of(2500);
  public static final Time SHOOTER_SHOT_PERIOD = Seconds.of(0.18);
}
