package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class IntakeConstants {
  // Motor config
  public static final int ROLLER_MOTOR_ID = 17;
  public static final boolean ROLLER_INVERTED = false;
  // 70 A, as in the 2026RobotProject (MAX_VORTEX_CURRENT). 30 A, then 40 A, left the roller
  // underpowered: it stalled starting next to a ball and bogged down while intaking.
  public static final Current ROLLER_CURRENT_LIMIT = Amps.of(70);

  // Voltages
  public static final Voltage INTAKE_RUN_VOLTAGE = Volts.of(11);
  public static final Voltage INTAKE_EJECT_VOLTAGE = Volts.of(-10);

  // Roller sim constant (RPM per volt)
  public static final double ROLLER_RPM_PER_VOLT = 124.075;

  public static final double FUEL_DIAMETER = 0.15;
}
