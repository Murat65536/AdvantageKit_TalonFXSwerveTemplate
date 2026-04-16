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
    bottomRightMotorConfig.apply(topLeftMotorConfig).follow(topLeftMotor).inverted(true).flatten();
    bottomRightMotor.configure(
        bottomRightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.velocity = RPM.of(encoder.getVelocity());
    inputs.voltageOut = Volts.of(topLeftMotor.getAppliedOutput() * topLeftMotor.getBusVoltage());
    inputs.currentOut = Amps.of(topLeftMotor.getOutputCurrent() + topRightMotor.getOutputCurrent());
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
    topLeftMotor.setVoltage(voltage.in(Volts));
    topRightMotor.setVoltage(voltage.in(Volts));
    bottomLeftMotor.setVoltage(voltage.in(Volts));
    bottomRightMotor.setVoltage(voltage.in(Volts));
  }
}
