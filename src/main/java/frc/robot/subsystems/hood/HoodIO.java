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

  /** Call with the hood held at the steep hard stop: makes that position read MAX_HOOD_ANGLE. */
  default void zeroAtSteepStop() {}

  /**
   * Drive the hood open loop at {@code volts} (negative is steeper), bypassing position control,
   * until {@link #stopJog}. Works before the hood is zeroed.
   */
  default void startJog(Voltage volts) {}

  default void stopJog() {}

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
    /** Output cut by the stall detector; clears when the robot is disabled. */
    public boolean stallCutoff = false;
    /** Position the Spark reports, in hood degrees, with its own zero offset already applied. */
    public double reportedEncoderDeg = 0.0;
    /**
     * Zero offset stored in the Spark's flash (rotations), the same setting REV Hardware Client
     * shows. Read back from the Spark at boot and after each zero, so a power cycle that loses it
     * shows up here.
     */
    public double sparkZeroOffsetRot = 0.0;
    /** False while the Spark's zero offset is still the factory 0. */
    public boolean zeroed = false;
    /** A Zero Hood press is being applied and checked; position control pauses meanwhile. */
    public boolean zeroInProgress = false;

    public boolean jogging = false;
  }
}
