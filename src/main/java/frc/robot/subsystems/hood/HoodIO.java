package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {

  default void updateInputs(HoodIOInputs inputs) {}

  default void setTargetAngle(Angle angle) {}

  @AutoLog
  class HoodIOInputs {
    public Angle angle = Degrees.zero();
    public AngularVelocity velocity = DegreesPerSecond.zero();
    public Voltage appliedVoltage = Volts.zero();
    public Current current = Amps.zero();
    public Temperature temp = Celsius.zero();
    public boolean leftMotorConnected = false;
    public boolean rightMotorConnected = false;
    public boolean encoderConnected = false;
    public boolean atSetpoint = false;
  }
}
