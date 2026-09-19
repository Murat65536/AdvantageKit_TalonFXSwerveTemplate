package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  default void updateInputs(ShooterIOInputs inputs) {}

  /** Open-loop voltage (SysId / manual). Cancels any velocity setpoint. */
  default void setShooterVoltage(Voltage voltage) {}

  /** Closed-loop velocity setpoint. */
  default void setShooterVelocity(AngularVelocity velocity) {}

  /** Disable output and coast. */
  default void stop() {}

  @AutoLog
  class ShooterIOInputs {
    public Angle position = Rotations.zero();
    public AngularVelocity velocity = RPM.zero();
    public AngularVelocity velocitySetpoint = RPM.zero();
    public Voltage voltageOut = Volts.zero();
    public Current currentOut = Amps.zero();
    public Temperature temp = Celsius.zero();
    public boolean connected = false;
  }
}
