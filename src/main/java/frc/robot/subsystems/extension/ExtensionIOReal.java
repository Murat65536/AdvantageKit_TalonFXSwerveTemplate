package frc.robot.subsystems.extension;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.extension.ExtensionConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

/** Real hardware IO for extension driven by one NEO 1.1 on Spark Flex with limit switches. */
public class ExtensionIOReal implements ExtensionIO {
  private final SparkFlex motor = new SparkFlex(MOTOR_CAN_ID, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();
  private final SparkLimitSwitch forwardLimitSwitch = motor.getForwardLimitSwitch();
  private final SparkLimitSwitch reverseLimitSwitch = motor.getReverseLimitSwitch();

  private final Alert motorDisconnected =
      new Alert("Extension Spark Flex disconnected!", AlertType.kError);

  private Voltage requestedVoltage = Volts.zero();

  public ExtensionIOReal() {
    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake).smartCurrentLimit((int) MOTOR_CURRENT_LIMIT.in(Amps));
    config
        .limitSwitch
        .forwardLimitSwitchType(Type.kNormallyOpen)
        .forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotorAndSetPosition)
        .reverseLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchTriggerBehavior(Behavior.kStopMovingMotorAndSetPosition);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(ExtensionIOInputs inputs) {
    boolean forwardPressed = forwardLimitSwitch.isPressed();
    boolean reversePressed = reverseLimitSwitch.isPressed();

    double safeVolts = requestedVoltage.in(Volts);
    if ((safeVolts > 0.0 && forwardPressed) || (safeVolts < 0.0 && reversePressed)) {
      safeVolts = 0.0;
    }
    motor.setVoltage(safeVolts);

    inputs.position = Rotations.of(encoder.getPosition());
    inputs.velocity = RotationsPerSecond.of(encoder.getVelocity() / 60.0);
    inputs.appliedVoltage = Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
    inputs.current = Amps.of(motor.getOutputCurrent());
    inputs.temp = Celsius.of(motor.getMotorTemperature());
    inputs.connected = !motor.hasActiveFault();
    inputs.forwardLimitPressed = forwardPressed;
    inputs.reverseLimitPressed = reversePressed;

    motorDisconnected.set(!inputs.connected);
  }

  @Override
  public void setMotorVoltage(Voltage voltage) {
    requestedVoltage = voltage;
  }
}
