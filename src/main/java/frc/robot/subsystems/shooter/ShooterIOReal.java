package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

/** Real hardware IO for the shooter flywheels using Neo Vortex motors on Spark Flex controllers. */
public class ShooterIOReal implements ShooterIO {
  private final SparkFlex topLeftMotor = new SparkFlex(TOP_LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
  private final SparkFlex topRightMotor =
      new SparkFlex(TOP_RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);
  private final SparkFlex bottomLeftMotor =
      new SparkFlex(BOTTOM_LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
  private final SparkFlex bottomRightMotor =
      new SparkFlex(BOTTOM_RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);
  private final RelativeEncoder encoder = topLeftMotor.getEncoder();

  private final Alert shooterDisconnected =
      new Alert("Shooter Spark Flex disconnected!", AlertType.kError);
  private Voltage appliedVoltage = Volts.zero();
  private AngularVelocity velocitySetpoint = RPM.zero();

  public ShooterIOReal() {
    SparkFlexConfig topLeftMotorConfig = new SparkFlexConfig();
    topLeftMotorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit((int) CURRENT_LIMIT.in(Amps));
    topLeftMotor.configure(
        topLeftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkFlexConfig topRightMotorConfig = new SparkFlexConfig();
    topRightMotorConfig.apply(topLeftMotorConfig).follow(topLeftMotor).inverted(true);
    topRightMotor.configure(
        topRightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkFlexConfig bottomLeftMotorConfig = new SparkFlexConfig();
    bottomLeftMotorConfig.apply(topLeftMotorConfig).follow(topLeftMotor);
    bottomLeftMotor.configure(
        bottomLeftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkFlexConfig bottomRightMotorConfig = new SparkFlexConfig();
    bottomRightMotorConfig.apply(topLeftMotorConfig).follow(topLeftMotor).inverted(true);
    bottomRightMotor.configure(
        bottomRightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    double targetRpm = velocitySetpoint.in(RPM);
    double measuredRpm = encoder.getVelocity();
    double ffVolts = targetRpm * SHOOTER_VELOCITY_FF_VOLTS_PER_RPM;
    double feedbackVolts = (targetRpm - measuredRpm) * SHOOTER_VELOCITY_KP_VOLTS_PER_RPM;
    appliedVoltage = Volts.of(MathUtil.clamp(ffVolts + feedbackVolts, -12.0, 12.0));
    topLeftMotor.setVoltage(appliedVoltage.in(Volts));

    inputs.velocity = RPM.of(encoder.getVelocity());
    inputs.voltageOut = Volts.of(topLeftMotor.getAppliedOutput() * topLeftMotor.getBusVoltage());
    inputs.currentOut =
        Amps.of(
            topLeftMotor.getOutputCurrent()
                + topRightMotor.getOutputCurrent()
                + bottomLeftMotor.getOutputCurrent()
                + bottomRightMotor.getOutputCurrent());
    inputs.temp =
        Celsius.of(
            Math.max(
                Math.max(topLeftMotor.getMotorTemperature(), topRightMotor.getMotorTemperature()),
                Math.max(
                    bottomLeftMotor.getMotorTemperature(),
                    bottomRightMotor.getMotorTemperature())));
    inputs.connected =
        !topLeftMotor.hasActiveFault()
            && !topRightMotor.hasActiveFault()
            && !bottomLeftMotor.hasActiveFault()
            && !bottomRightMotor.hasActiveFault();

    shooterDisconnected.set(!inputs.connected);
  }

  @Override
  public void setShooterVoltage(Voltage voltage) {
    double targetRpm =
        MathUtil.clamp(voltage.in(Volts), -12.0, 12.0) / 12.0 * MAX_FLYWHEEL_VELOCITY.in(RPM);
    velocitySetpoint = RPM.of(targetRpm);
  }

  @Override
  public void setShooterVelocity(AngularVelocity velocity) {
    velocitySetpoint = velocity;
  }
}
