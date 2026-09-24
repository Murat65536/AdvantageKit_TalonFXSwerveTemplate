package frc.robot.subsystems.rollers;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * Simulated roller: a Neo Vortex spinning a small flywheel. Emulates the Spark's onboard velocity
 * loop (kV feedforward + proportional feedback). Gains are in VOLTS per RPM, matching REV's {@code
 * closedLoop.feedForward.kV()} / {@code pid()} convention used by {@link RollerIOSparkFlex}, so the
 * same constants behave the same way in sim and on the robot.
 */
public class RollerIOSim implements RollerIO {
  private static final double LOOP_PERIOD_SECONDS = 0.02;
  private static final double ROLLER_MOI_KG_M2 = 0.0005;

  private final FlywheelSim sim;
  private final DCMotor motor = DCMotor.getNeoVortex(1);
  private final double kP;
  private final double kV;

  private AngularVelocity velocitySetpoint = RPM.zero();
  private boolean closedLoop = false;
  private double appliedVolts = 0.0;

  public RollerIOSim(RollerConfig config) {
    sim = new FlywheelSim(LinearSystemId.createFlywheelSystem(motor, ROLLER_MOI_KG_M2, 1.0), motor);
    kP = config.kP();
    kV = config.kV();
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    if (closedLoop) {
      double targetRpm = velocitySetpoint.in(RPM);
      double measuredRpm = sim.getAngularVelocityRPM();
      // kV and kP are volts per RPM (see class doc), so this sums directly to volts.
      double volts = kV * targetRpm + kP * (targetRpm - measuredRpm);
      appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    }
    sim.setInputVoltage(appliedVolts);
    sim.update(LOOP_PERIOD_SECONDS);

    inputs.velocity = RPM.of(sim.getAngularVelocityRPM());
    inputs.velocitySetpoint = velocitySetpoint;
    inputs.appliedVoltage = Volts.of(appliedVolts);
    inputs.current = Amps.of(Math.abs(sim.getCurrentDrawAmps()));
    inputs.temp = Celsius.zero();
    inputs.connected = true;
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    velocitySetpoint = velocity;
    closedLoop = true;
  }

  @Override
  public void setVoltage(Voltage voltage) {
    velocitySetpoint = RPM.zero();
    closedLoop = false;
    appliedVolts = MathUtil.clamp(voltage.in(Volts), -12.0, 12.0);
  }

  @Override
  public void stop() {
    velocitySetpoint = RPM.zero();
    closedLoop = false;
    appliedVolts = 0.0;
  }
}
