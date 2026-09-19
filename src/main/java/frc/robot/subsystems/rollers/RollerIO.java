package frc.robot.subsystems.rollers;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/** Generic IO for a single velocity-controlled roller (feeder, indexer, kicker). */
public interface RollerIO {

  default void updateInputs(RollerIOInputs inputs) {}

  /** Run closed-loop velocity control at the given setpoint. */
  default void setVelocity(AngularVelocity velocity) {}

  /** Run open-loop at the given voltage (used for SysId / manual override). */
  default void setVoltage(Voltage voltage) {}

  /** Disable output and let the roller coast. */
  default void stop() {}

  @AutoLog
  class RollerIOInputs {
    public AngularVelocity velocity = RPM.zero();
    public AngularVelocity velocitySetpoint = RPM.zero();
    public Voltage appliedVoltage = Volts.zero();
    public Current current = Amps.zero();
    public Temperature temp = Celsius.zero();
    public boolean connected = false;
  }
}
