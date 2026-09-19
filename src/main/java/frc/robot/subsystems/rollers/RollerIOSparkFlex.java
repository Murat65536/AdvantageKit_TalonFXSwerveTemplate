package frc.robot.subsystems.rollers;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

/**
 * Roller driven by a Neo Vortex on a Spark Flex. Velocity closed-loop runs onboard the Spark (PID +
 * kV feedforward) so it is unaffected by RIO loop timing.
 */
public class RollerIOSparkFlex implements RollerIO {
  private final SparkFlex motor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController controller;
  private final Alert disconnectedAlert;

  private AngularVelocity velocitySetpoint = RPM.zero();

  public RollerIOSparkFlex(RollerConfig config) {
    motor = new SparkFlex(config.canId(), MotorType.kBrushless);
    encoder = motor.getEncoder();
    controller = motor.getClosedLoopController();
    disconnectedAlert = new Alert(config.name() + " Spark Flex disconnected!", AlertType.kError);

    SparkFlexConfig sparkConfig = new SparkFlexConfig();
    sparkConfig
        .idleMode(IdleMode.kCoast)
        .inverted(config.inverted())
        .smartCurrentLimit((int) config.currentLimit().in(Amps));
    sparkConfig.closedLoop.pid(config.kP(), config.kI(), config.kD()).feedForward.kV(config.kV());
    // Smooth the velocity measurement the onboard loop closes on.
    sparkConfig.encoder.quadratureAverageDepth(5).quadratureMeasurementPeriod(10);
    motor.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.velocity = RPM.of(encoder.getVelocity());
    inputs.velocitySetpoint = velocitySetpoint;
    inputs.appliedVoltage = Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
    inputs.current = Amps.of(motor.getOutputCurrent());
    inputs.temp = Celsius.of(motor.getMotorTemperature());
    inputs.connected = motor.getLastError() == REVLibError.kOk;
    disconnectedAlert.set(!inputs.connected);
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    velocitySetpoint = velocity;
    controller.setSetpoint(velocity.in(RPM), ControlType.kVelocity);
  }

  @Override
  public void setVoltage(Voltage voltage) {
    velocitySetpoint = RPM.zero();
    motor.setVoltage(voltage.in(Volts));
  }

  @Override
  public void stop() {
    velocitySetpoint = RPM.zero();
    motor.stopMotor();
  }
}
