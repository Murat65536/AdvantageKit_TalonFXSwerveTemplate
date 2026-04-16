package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IO/Shooter", inputs);
    Logger.recordOutput(
        "Subsystems/Shooter/command",
        getCurrentCommand() == null ? "none" : getCurrentCommand().getName());
  }

  /** Spin up the flywheel at full voltage. Stops when released. */
  public Command shoot() {
    return Commands.startEnd(
        () -> io.setShooterVoltage(SHOOT_VOLTAGE), () -> io.setShooterVoltage(Volts.zero()), this);
  }

  /** Reverse the flywheel at low voltage (e.g. for unjamming). Stops when released. */
  public Command reverse() {
    return Commands.startEnd(
        () -> io.setShooterVoltage(REVERSE_VOLTAGE),
        () -> io.setShooterVoltage(Volts.zero()),
        this);
  }
}
