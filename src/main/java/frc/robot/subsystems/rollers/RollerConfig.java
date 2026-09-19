package frc.robot.subsystems.rollers;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

/**
 * Hardware and tuning description for a single Spark-driven roller.
 *
 * @param name human-readable name used in alerts
 * @param canId Spark CAN ID
 * @param inverted motor inversion
 * @param currentLimit smart current limit
 * @param kP onboard velocity loop proportional gain (duty cycle per RPM of error)
 * @param kI onboard velocity loop integral gain
 * @param kD onboard velocity loop derivative gain
 * @param kV onboard velocity feedforward (duty cycle per RPM)
 * @param tolerance velocity tolerance for {@code atSetpoint}
 */
public record RollerConfig(
    String name,
    int canId,
    boolean inverted,
    Current currentLimit,
    double kP,
    double kI,
    double kD,
    double kV,
    AngularVelocity tolerance) {}
