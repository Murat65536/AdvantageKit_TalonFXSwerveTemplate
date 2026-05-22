package frc.robot.subsystems.extension;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.extension.ExtensionConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Extension subsystem for extending/retracting the intake mechanism with limit-switch protection.
 */
public class Extension extends SubsystemBase {
  private final ExtensionIO io;
  private final ExtensionIOInputsAutoLogged inputs = new ExtensionIOInputsAutoLogged();

  public Extension(ExtensionIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IO/Extension", inputs);
    Pose3d componentPose = getComponentPose();
    Logger.recordOutput("Subsystems/Extension/ComponentPose", componentPose);
    Logger.recordOutput(
        "Subsystems/Extension/command",
        getCurrentCommand() == null ? "none" : getCurrentCommand().getName());
  }

  /** Extend while held. Motion halts automatically at the forward limit switch. */
  public Command extend() {
    return Commands.startEnd(
        () -> io.setMotorVoltage(EXTEND_VOLTAGE), () -> io.setMotorVoltage(Volts.zero()), this);
  }

  /** Retract while held. Motion halts automatically at the reverse limit switch. */
  public Command retract() {
    return Commands.startEnd(
        () -> io.setMotorVoltage(RETRACT_VOLTAGE), () -> io.setMotorVoltage(Volts.zero()), this);
  }

  /** True when the extension has reached the forward limit switch. */
  public boolean isFullyExtended() {
    return inputs.forwardLimitPressed;
  }

  public BooleanSupplier fullyExtendedSupplier() {
    return this::isFullyExtended;
  }

  private Pose3d getComponentPose() {
    double extensionMeters =
        MathUtil.clamp(
            inputs.position.in(Rotations) * COMPONENT_METERS_PER_ROTATION,
            MIN_POSITION_ROT * COMPONENT_METERS_PER_ROTATION,
            MAX_POSITION_ROT * COMPONENT_METERS_PER_ROTATION);
    double slopeRad = Math.toRadians(COMPONENT_SLOPE_DEG);
    double xDeltaMeters = extensionMeters * Math.cos(slopeRad);
    double zDeltaMeters = -extensionMeters * Math.sin(slopeRad);
    return new Pose3d(xDeltaMeters, 0, zDeltaMeters, new Rotation3d());
  }
}
