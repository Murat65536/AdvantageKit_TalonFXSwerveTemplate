package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IO/Intake", inputs);
    Logger.recordOutput(
        "Subsystems/Intake/command",
        getCurrentCommand() == null ? "none" : getCurrentCommand().getName());
  }

  /** Run the roller to collect game pieces. Stops when released. */
  public Command intake() {
    return Commands.startEnd(
        () -> io.setRollerVoltage(INTAKE_RUN_VOLTAGE),
        () -> io.setRollerVoltage(Volts.of(0)),
        this);
  }

  /** Run the roller only while intake is allowed (e.g. extension fully extended). */
  public Command intake(BooleanSupplier canIntake) {
    return Commands.runEnd(
        () -> io.setRollerVoltage(canIntake.getAsBoolean() ? INTAKE_RUN_VOLTAGE : Volts.zero()),
        () -> io.setRollerVoltage(Volts.zero()),
        this);
  }

  /** Reverse the roller to eject game pieces. Stops when released. */
  public Command outtake() {
    return Commands.startEnd(
        () -> io.setRollerVoltage(INTAKE_EJECT_VOLTAGE),
        () -> io.setRollerVoltage(Volts.of(0)),
        this);
  }

  public int getStoredGamePieces() {
    return io.getStoredGamePieces();
  }

  public boolean consumeGamePiece() {
    return io.consumeGamePiece();
  }
}
