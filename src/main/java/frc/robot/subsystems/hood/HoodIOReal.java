package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.hood.HoodConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

/** Real hardware IO for the hood using two NEO 550s on Spark MAX and a CANcoder angle sensor. */
public class HoodIOReal implements HoodIO {
  private final SparkMax leftMotor = new SparkMax(LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
  private final SparkMax rightMotor = new SparkMax(RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);
  private final CANcoder cancoder = new CANcoder(HOOD_CANCODER_CAN_ID);

  private final StatusSignal<Angle> cancoderAbsolutePosition = cancoder.getAbsolutePosition();
  private final StatusSignal<AngularVelocity> cancoderVelocity = cancoder.getVelocity();

  private final Alert leftMotorDisconnected =
      new Alert("Hood left Spark MAX disconnected!", AlertType.kError);
  private final Alert rightMotorDisconnected =
      new Alert("Hood right Spark MAX disconnected!", AlertType.kError);
  private final Alert cancoderDisconnected =
      new Alert("Hood CANcoder disconnected!", AlertType.kError);

  private Angle targetAngle = MIN_HOOD_ANGLE;

  public HoodIOReal() {
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig
        .idleMode(IdleMode.kBrake)
        .inverted(LEFT_MOTOR_INVERTED)
        .smartCurrentLimit((int) MOTOR_CURRENT_LIMIT.in(Amps));
    leftMotor.configure(
        leftConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig
        .apply(leftConfig)
        .follow(leftMotor, RIGHT_MOTOR_FOLLOW_INVERTED)
        .smartCurrentLimit((int) MOTOR_CURRENT_LIMIT.in(Amps));
    rightMotor.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    boolean cancoderOk =
        BaseStatusSignal.refreshAll(cancoderAbsolutePosition, cancoderVelocity).isOK();
    double angleDeg =
        Units.rotationsToDegrees(cancoderAbsolutePosition.getValueAsDouble())
            - HOOD_CANCODER_ZERO_OFFSET.in(Degrees);
    double velocityDegPerSec = Units.rotationsToDegrees(cancoderVelocity.getValueAsDouble());
    double errorDeg = targetAngle.in(Degrees) - angleDeg;
    double controlVolts =
        MathUtil.clamp(
            errorDeg * HOOD_KP_VOLTS_PER_DEG,
            -MAX_CONTROL_VOLTAGE.in(Volts),
            MAX_CONTROL_VOLTAGE.in(Volts));
    leftMotor.setVoltage(controlVolts);

    inputs.angle = Degrees.of(angleDeg);
    inputs.velocity = DegreesPerSecond.of(velocityDegPerSec);
    inputs.appliedVoltage = Volts.of(leftMotor.getAppliedOutput() * leftMotor.getBusVoltage());
    inputs.current = Amps.of(leftMotor.getOutputCurrent() + rightMotor.getOutputCurrent());
    inputs.temp =
        Celsius.of(Math.max(leftMotor.getMotorTemperature(), rightMotor.getMotorTemperature()));
    inputs.leftMotorConnected = !leftMotor.hasActiveFault();
    inputs.rightMotorConnected = !rightMotor.hasActiveFault();
    inputs.cancoderConnected = cancoderOk;
    inputs.atSetpoint = Math.abs(errorDeg) <= HOOD_ANGLE_TOLERANCE.in(Degrees);

    leftMotorDisconnected.set(!inputs.leftMotorConnected);
    rightMotorDisconnected.set(!inputs.rightMotorConnected);
    cancoderDisconnected.set(!inputs.cancoderConnected);
  }

  @Override
  public void setTargetAngle(Angle angle) {
    targetAngle =
        Degrees.of(
            MathUtil.clamp(
                angle.in(Degrees), MIN_HOOD_ANGLE.in(Degrees), MAX_HOOD_ANGLE.in(Degrees)));
  }
}
