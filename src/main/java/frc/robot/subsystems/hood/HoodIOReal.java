package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.hood.HoodConstants.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

/** Real hardware IO for the hood using two NEO 550s on Spark MAX and motor absolute encoder. */
public class HoodIOReal implements HoodIO {
  private final SparkMax leftMotor = new SparkMax(LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
  private final SparkMax rightMotor = new SparkMax(RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);
  private final AbsoluteEncoder absoluteEncoder = leftMotor.getAbsoluteEncoder();

  private final Alert leftMotorDisconnected =
      new Alert("Hood left Spark MAX disconnected!", AlertType.kError);
  private final Alert rightMotorDisconnected =
      new Alert("Hood right Spark MAX disconnected!", AlertType.kError);
  private final Alert encoderDisconnected =
      new Alert("Hood motor absolute encoder disconnected!", AlertType.kError);

  private Angle targetAngle = MIN_HOOD_ANGLE;

  public HoodIOReal() {
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.idleMode(IdleMode.kBrake).smartCurrentLimit((int) MOTOR_CURRENT_LIMIT.in(Amps));
    leftMotor.configure(
        leftConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig
        .apply(leftConfig)
        .follow(leftMotor, true)
        .smartCurrentLimit((int) MOTOR_CURRENT_LIMIT.in(Amps));
    rightMotor.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    double angleDeg =
        Units.rotationsToDegrees(absoluteEncoder.getPosition())
            - HOOD_ABSOLUTE_ENCODER_ZERO_OFFSET.in(Degrees);
    double velocityDegPerSec = Units.rotationsToDegrees(absoluteEncoder.getVelocity());
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
    inputs.encoderConnected = inputs.leftMotorConnected;
    inputs.atSetpoint = Math.abs(errorDeg) <= HOOD_ANGLE_TOLERANCE.in(Degrees);

    leftMotorDisconnected.set(!inputs.leftMotorConnected);
    rightMotorDisconnected.set(!inputs.rightMotorConnected);
    encoderDisconnected.set(!inputs.encoderConnected);
  }

  @Override
  public void setTargetAngle(Angle angle) {
    targetAngle =
        Degrees.of(
            MathUtil.clamp(
                angle.in(Degrees), MIN_HOOD_ANGLE.in(Degrees), MAX_HOOD_ANGLE.in(Degrees)));
  }
}
