package frc.robot.subsystems.extension;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.extension.ExtensionConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

/**
 * Real hardware IO for the hopper extension: one NEO on a Spark MAX with a reverse limit switch.
 */
public class ExtensionIOReal implements ExtensionIO {
  private final SparkMax motor = new SparkMax(MOTOR_CAN_ID, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();
  private final SparkLimitSwitch reverseLimitSwitch = motor.getReverseLimitSwitch();

  private final Alert motorDisconnected =
      new Alert("Extension Spark MAX disconnected!", AlertType.kError);

  private Voltage requestedVoltage = Volts.zero();

  public ExtensionIOReal() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake).smartCurrentLimit((int) MOTOR_CURRENT_LIMIT.in(Amps));
    config
        .limitSwitch
        .reverseLimitSwitchEnabled(true)
        .reverseLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchTriggerBehavior(Behavior.kStopMovingMotorAndSetPosition)
        .reverseLimitSwitchPosition(MIN_POSITION_ROT)
        .limitSwitchPositionSensor(FeedbackSensor.kPrimaryEncoder);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(ExtensionIOInputs inputs) {
    boolean reversePressed = reverseLimitSwitch.isPressed();

    // The Spark stops the motor at the switch on its own; also refuse to command into it.
    double safeVolts = requestedVoltage.in(Volts);
    if (safeVolts < 0.0 && reversePressed) {
      safeVolts = 0.0;
    }
    motor.setVoltage(safeVolts);

    inputs.position = Rotations.of(encoder.getPosition());
    inputs.velocity = RotationsPerSecond.of(encoder.getVelocity() / 60.0);
    inputs.appliedVoltage = Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
    inputs.current = Amps.of(motor.getOutputCurrent());
    inputs.temp = Celsius.of(motor.getMotorTemperature());
    inputs.connected = motor.getLastError() == REVLibError.kOk;
    inputs.reverseLimitPressed = reversePressed;

    motorDisconnected.set(!inputs.connected);
  }

  @Override
  public void setMotorVoltage(Voltage voltage) {
    requestedVoltage = voltage;
  }
}
