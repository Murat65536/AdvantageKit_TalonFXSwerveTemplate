package frc.robot.subsystems.extension;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.extension.ExtensionConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Voltage;

/**
 * Simulated IO for the extension with a software-emulated reverse limit switch and a hard stop at
 * the sim travel limit.
 */
public class ExtensionIOSim implements ExtensionIO {
  private Voltage requestedVoltage = Volts.zero();
  private double positionRot = MIN_POSITION_ROT;
  private double velocityRps = 0.0;

  @Override
  public void updateInputs(ExtensionIOInputs inputs) {
    boolean reversePressed = positionRot <= MIN_POSITION_ROT;
    boolean atHardStop = positionRot >= SIM_MAX_POSITION_ROT;

    double safeVolts = requestedVoltage.in(Volts);
    if ((safeVolts < 0.0 && reversePressed) || (safeVolts > 0.0 && atHardStop)) {
      safeVolts = 0.0;
    }

    velocityRps = safeVolts * SIM_RPS_PER_VOLT;
    positionRot =
        MathUtil.clamp(positionRot + velocityRps * 0.02, MIN_POSITION_ROT, SIM_MAX_POSITION_ROT);

    inputs.position = Rotations.of(positionRot);
    inputs.velocity = RotationsPerSecond.of(velocityRps);
    inputs.appliedVoltage = Volts.of(safeVolts);
    inputs.current = Amps.of(Math.abs(safeVolts) * 1.3);
    inputs.temp = Celsius.zero();
    inputs.connected = true;
    inputs.reverseLimitPressed = positionRot <= MIN_POSITION_ROT;
  }

  @Override
  public void setMotorVoltage(Voltage voltage) {
    requestedVoltage = voltage;
  }
}
