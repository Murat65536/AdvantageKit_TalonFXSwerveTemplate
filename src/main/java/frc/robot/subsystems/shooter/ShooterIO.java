package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setShooterVoltage(Voltage voltage) {}

  default void setShooterVelocity(AngularVelocity velocity) {}

  @AutoLog
  class ShooterIOInputs {
    public AngularVelocity velocity = RPM.zero();
    public Voltage voltageOut = Volts.zero();
    public Current currentOut = Amps.zero();
    public Temperature temp = Celsius.zero();
    public boolean connected = false;
  }
}
