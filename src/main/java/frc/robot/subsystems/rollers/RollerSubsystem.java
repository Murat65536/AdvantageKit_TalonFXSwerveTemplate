package frc.robot.subsystems.rollers;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Base class for a single velocity-controlled roller subsystem. Subclasses supply the IO, the
 * logging key, and the tolerance; they expose named commands built on {@link #runVelocity}.
 */
public abstract class RollerSubsystem extends SubsystemBase {
  private final RollerIO io;
  private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();
  private final String logKey;
  private final AngularVelocity tolerance;

  private AngularVelocity targetVelocity = RPM.zero();

  protected RollerSubsystem(String name, RollerIO io, AngularVelocity tolerance) {
    this.io = io;
    this.logKey = name;
    this.tolerance = tolerance;
    setName(name);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IO/" + logKey, inputs);
    Logger.recordOutput("Subsystems/" + logKey + "/TargetRpm", targetVelocity.in(RPM));
    Logger.recordOutput("Subsystems/" + logKey + "/AtSetpoint", atSetpoint());
    Logger.recordOutput(
        "Subsystems/" + logKey + "/command",
        getCurrentCommand() == null ? "none" : getCurrentCommand().getName());
  }

  /** Runs the roller at a velocity setpoint until the command ends, then stops. */
  public Command runVelocity(AngularVelocity velocity) {
    return Commands.startEnd(() -> setVelocity(velocity), this::stop, this);
  }

  /** Runs open-loop at a voltage until the command ends, then stops. */
  public Command runVoltage(Voltage voltage) {
    return Commands.startEnd(() -> io.setVoltage(voltage), this::stop, this);
  }

  public void setVelocity(AngularVelocity velocity) {
    targetVelocity = velocity;
    io.setVelocity(velocity);
  }

  public void stop() {
    targetVelocity = RPM.zero();
    io.stop();
  }

  public AngularVelocity getVelocity() {
    return inputs.velocity;
  }

  public AngularVelocity getTargetVelocity() {
    return targetVelocity;
  }

  /** True when a nonzero setpoint is active and the measured velocity is within tolerance. */
  public boolean atSetpoint() {
    return !targetVelocity.isEquivalent(RPM.zero())
        && MathUtil.isNear(targetVelocity.in(RPM), inputs.velocity.in(RPM), tolerance.in(RPM));
  }

  /** True when the roller is being commanded forward (positive setpoint). */
  public boolean isRunningForward() {
    return targetVelocity.gt(RPM.zero());
  }
}
