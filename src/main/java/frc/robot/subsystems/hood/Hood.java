package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.hood.HoodConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  public Hood(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IO/Hood", inputs);
    Logger.recordOutput(
        "Subsystems/Hood/command",
        getCurrentCommand() == null ? "none" : getCurrentCommand().getName());
  }

  public void setTargetAngle(Angle angle) {
    io.setTargetAngle(
        Degrees.of(
            MathUtil.clamp(
                angle.in(Degrees), MIN_HOOD_ANGLE.in(Degrees), MAX_HOOD_ANGLE.in(Degrees))));
  }

  public Angle getAngle() {
    return inputs.angle;
  }

  public boolean atSetpoint() {
    return inputs.atSetpoint;
  }
}
