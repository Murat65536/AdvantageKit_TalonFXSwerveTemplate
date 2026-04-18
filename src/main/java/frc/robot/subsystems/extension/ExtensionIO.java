package frc.robot.subsystems.extension;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ExtensionIO {

  default void updateInputs(ExtensionIOInputs inputs) {}

  default void setMotorVoltage(Voltage voltage) {}

  @AutoLog
  class ExtensionIOInputs {
    public Angle position = Rotations.zero();
    public AngularVelocity velocity = RotationsPerSecond.zero();
    public Voltage appliedVoltage = Volts.zero();
    public Current current = Amps.zero();
    public Temperature temp = Celsius.zero();
    public boolean connected = false;
    public boolean forwardLimitPressed = false;
    public boolean reverseLimitPressed = false;
  }
}
