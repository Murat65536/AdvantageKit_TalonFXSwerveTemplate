package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.hood.HoodConstants.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

/**
 * Real hardware IO for the hood: two NEO 550s on Spark MAX (right follows left, inverted) and an
 * absolute encoder on the left Spark. Position control is a proportional loop on the RIO.
 */
public class HoodIOReal implements HoodIO {
  private final SparkMax leftMotor = new SparkMax(LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
  private final SparkMax rightMotor = new SparkMax(RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);
  private final AbsoluteEncoder absoluteEncoder = leftMotor.getAbsoluteEncoder();

  private final Alert leftMotorDisconnected =
      new Alert("Hood left Spark MAX disconnected!", AlertType.kError);
  private final Alert rightMotorDisconnected =
      new Alert("Hood right Spark MAX disconnected!", AlertType.kError);

  private Angle targetAngle = MIN_HOOD_ANGLE;

  public HoodIOReal() {
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.idleMode(IdleMode.kBrake).smartCurrentLimit((int) MOTOR_CURRENT_LIMIT.in(Amps));
    leftConfig
        .absoluteEncoder
        .inverted(ABSOLUTE_ENCODER_INVERTED)
        .positionConversionFactor(ENCODER_POSITION_CONVERSION_FACTOR)
        .velocityConversionFactor(ENCODER_POSITION_CONVERSION_FACTOR / 60.0);
    leftMotor.configure(
        leftConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) MOTOR_CURRENT_LIMIT.in(Amps))
        .follow(leftMotor, true);
    rightMotor.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    double encoderPositionDeg = absoluteEncoder.getPosition();
    double encoderVelocityDegPerSec = absoluteEncoder.getVelocity();
    Angle angle = encoderPositionToAngle(encoderPositionDeg);

    // Error in encoder units, wrapped to the encoder's period so the discontinuity in the
    // absolute encoder (one rev = 2x hood travel) never produces a runaway command.
    double targetEncoderDeg = MAX_HOOD_ANGLE.minus(targetAngle).in(Degrees);
    double encoderErrorDeg =
        MathUtil.inputModulus(
            targetEncoderDeg - encoderPositionDeg,
            -ENCODER_POSITION_CONVERSION_FACTOR / 2.0,
            ENCODER_POSITION_CONVERSION_FACTOR / 2.0);
    boolean atSetpoint = Math.abs(encoderErrorDeg) <= HOOD_ANGLE_TOLERANCE.in(Degrees);
    double controlVolts =
        atSetpoint
            ? 0.0
            : MathUtil.clamp(
                encoderErrorDeg * HOOD_KP_VOLTS_PER_DEG,
                -MAX_CONTROL_VOLTAGE.in(Volts),
                MAX_CONTROL_VOLTAGE.in(Volts));
    leftMotor.setVoltage(controlVolts);

    inputs.angle = angle;
    // Encoder increases as the hood retracts, so hood angle velocity is the negative.
    inputs.velocity = DegreesPerSecond.of(-encoderVelocityDegPerSec);
    inputs.appliedVoltage = Volts.of(leftMotor.getAppliedOutput() * leftMotor.getBusVoltage());
    inputs.current = Amps.of(leftMotor.getOutputCurrent() + rightMotor.getOutputCurrent());
    inputs.temp =
        Celsius.of(Math.max(leftMotor.getMotorTemperature(), rightMotor.getMotorTemperature()));
    inputs.leftMotorConnected = leftMotor.getLastError() == REVLibError.kOk;
    inputs.rightMotorConnected = rightMotor.getLastError() == REVLibError.kOk;
    inputs.encoderConnected = !leftMotor.getFaults().sensor;
    inputs.atSetpoint = atSetpoint;

    leftMotorDisconnected.set(!inputs.leftMotorConnected);
    rightMotorDisconnected.set(!inputs.rightMotorConnected);
  }

  @Override
  public void setTargetAngle(Angle angle) {
    targetAngle =
        Degrees.of(
            MathUtil.clamp(
                angle.in(Degrees), MIN_HOOD_ANGLE.in(Degrees), MAX_HOOD_ANGLE.in(Degrees)));
  }
}
