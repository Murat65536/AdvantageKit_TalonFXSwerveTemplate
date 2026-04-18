package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.hood.HoodConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;

/** Simulated IO for hood angle control. */
public class HoodIOSim implements HoodIO {
  private Angle targetAngle = MIN_HOOD_ANGLE;
  private double currentAngleDeg = MIN_HOOD_ANGLE.in(Degrees);
  private double currentVelocityDegPerSec = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    double errorDeg = targetAngle.in(Degrees) - currentAngleDeg;
    currentVelocityDegPerSec =
        MathUtil.clamp(
            errorDeg * HOOD_SIM_RESPONSE_PER_SECOND,
            -HOOD_SIM_MAX_VELOCITY.in(DegreesPerSecond),
            HOOD_SIM_MAX_VELOCITY.in(DegreesPerSecond));
    currentAngleDeg =
        MathUtil.clamp(
            currentAngleDeg + currentVelocityDegPerSec * 0.02,
            MIN_HOOD_ANGLE.in(Degrees),
            MAX_HOOD_ANGLE.in(Degrees));
    appliedVolts = MathUtil.clamp(errorDeg * HOOD_SIM_KP_VOLTS_PER_DEG, -12.0, 12.0);

    inputs.angle = Degrees.of(currentAngleDeg);
    inputs.velocity = DegreesPerSecond.of(currentVelocityDegPerSec);
    inputs.appliedVoltage = Volts.of(appliedVolts);
    inputs.current = Amps.of(Math.abs(appliedVolts) * 0.9);
    inputs.temp = Celsius.of(30.0 + Math.abs(appliedVolts));
    inputs.leftMotorConnected = true;
    inputs.rightMotorConnected = true;
    inputs.encoderConnected = true;
    inputs.atSetpoint = Math.abs(errorDeg) <= HOOD_ANGLE_TOLERANCE.in(Degrees);
  }

  @Override
  public void setTargetAngle(Angle angle) {
    targetAngle =
        Degrees.of(
            MathUtil.clamp(
                angle.in(Degrees), MIN_HOOD_ANGLE.in(Degrees), MAX_HOOD_ANGLE.in(Degrees)));
  }
}
