package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setFlywheelVoltages(Voltage topVoltage, Voltage bottomVoltage) {}

  @AutoLog
  class ShooterIOInputs {
    public AngularVelocity topFlywheelVelocity = RPM.zero();
    public AngularVelocity bottomFlywheelVelocity = RPM.zero();
    public Voltage topFlywheelVoltageOut = Volts.zero();
    public Voltage bottomFlywheelVoltageOut = Volts.zero();
    public Current topFlywheelCurrentOut = Amps.zero();
    public Current bottomFlywheelCurrentOut = Amps.zero();
    public Temperature topFlywheelTemp = Celsius.zero();
    public Temperature bottomFlywheelTemp = Celsius.zero();
    public boolean topFlywheelConnected = false;
    public boolean bottomFlywheelConnected = false;
  }
}
