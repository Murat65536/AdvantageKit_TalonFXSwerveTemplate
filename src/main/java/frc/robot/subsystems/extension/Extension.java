package frc.robot.subsystems.extension;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.extension.ExtensionConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Hopper extension. Extends open-loop on a timer (there is no forward limit switch) and retracts
 * until the reverse limit switch trips.
 */
public class Extension extends SubsystemBase {
  private final ExtensionIO io;
  private final ExtensionIOInputsAutoLogged inputs = new ExtensionIOInputsAutoLogged();

  /** Set when a timed extend completes; cleared whenever the extension is driven inward. */
  private boolean fullyExtended = false;

  public Extension(ExtensionIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IO/Extension", inputs);
    Logger.recordOutput("Subsystems/Extension/ComponentPose", getComponentPose());
    Logger.recordOutput("Subsystems/Extension/FullyExtended", fullyExtended);
    Logger.recordOutput(
        "Subsystems/Extension/command",
        getCurrentCommand() == null ? "none" : getCurrentCommand().getName());
  }

  /** Extend for {@link ExtensionConstants#EXTEND_TIME}, then stop and mark fully extended. */
  public Command extend() {
    return Commands.startEnd(
            () -> io.setMotorVoltage(EXTEND_VOLTAGE), () -> io.setMotorVoltage(Volts.zero()), this)
        .withTimeout(EXTEND_TIME.in(Seconds))
        // Only a completed (not interrupted) extend counts as fully extended.
        .andThen(Commands.runOnce(() -> fullyExtended = true))
        .withName("ExtensionExtend");
  }

  /** Retract until the reverse limit switch is pressed. */
  public Command retract() {
    return Commands.startEnd(
            () -> {
              fullyExtended = false;
              io.setMotorVoltage(RETRACT_VOLTAGE);
            },
            () -> io.setMotorVoltage(Volts.zero()),
            this)
        .until(this::isRetracted)
        .withName("ExtensionRetract");
  }

  /**
   * Slowly pull the hopper inward while feeding fuel to the shooter, ramping the voltage over
   * {@link ExtensionConstants#CREEP_RAMP_TIME}. Ends when the reverse limit switch trips.
   */
  public Command creepIn() {
    Timer timer = new Timer();
    return Commands.runEnd(
            () -> {
              double t = MathUtil.clamp(timer.get() / CREEP_RAMP_TIME.in(Seconds), 0.0, 1.0);
              double volts =
                  MathUtil.interpolate(
                      CREEP_IN_START_VOLTAGE.in(Volts), CREEP_IN_END_VOLTAGE.in(Volts), t);
              io.setMotorVoltage(Volts.of(volts));
            },
            () -> io.setMotorVoltage(Volts.zero()),
            this)
        .beforeStarting(
            () -> {
              fullyExtended = false;
              timer.restart();
            })
        .until(this::isRetracted)
        .withName("ExtensionCreepIn");
  }

  /** Hold position (no output). */
  public Command stop() {
    return Commands.runOnce(() -> io.setMotorVoltage(Volts.zero()), this).withName("ExtensionStop");
  }

  /** True once a timed extend has completed and the extension has not since been driven in. */
  public boolean isFullyExtended() {
    return fullyExtended;
  }

  public boolean isRetracted() {
    return inputs.reverseLimitPressed;
  }

  public BooleanSupplier fullyExtendedSupplier() {
    return this::isFullyExtended;
  }

  private Pose3d getComponentPose() {
    double extensionMeters =
        MathUtil.clamp(
            inputs.position.in(Rotations) * COMPONENT_METERS_PER_ROTATION,
            0.0,
            COMPONENT_MAX_METERS);
    double slopeRad = Math.toRadians(COMPONENT_SLOPE_DEG);
    return new Pose3d(
        COMPONENT_ZERO_TRANSLATION.plus(
            new Translation3d(
                extensionMeters * Math.cos(slopeRad), 0, -extensionMeters * Math.sin(slopeRad))),
        new Rotation3d());
  }
}
