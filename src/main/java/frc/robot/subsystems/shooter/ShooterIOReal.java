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
  private final SparkFlex topFlywheelMotor =
      new SparkFlex(TOP_FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
  private final SparkFlex bottomFlywheelMotor =
      new SparkFlex(BOTTOM_FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
  private final RelativeEncoder topEncoder = topFlywheelMotor.getEncoder();
  private final RelativeEncoder bottomEncoder = bottomFlywheelMotor.getEncoder();

  private final Alert topFlywheelDisconnected =
      new Alert("Shooter top flywheel Spark Flex disconnected!", AlertType.kError);
  private final Alert bottomFlywheelDisconnected =
      new Alert("Shooter bottom flywheel Spark Flex disconnected!", AlertType.kError);

  public ShooterIOReal() {
    SparkFlexConfig topConfig = new SparkFlexConfig();
    topConfig
        .idleMode(IdleMode.kCoast)
        .inverted(TOP_FLYWHEEL_INVERTED)
        .smartCurrentLimit((int) FLYWHEEL_CURRENT_LIMIT.in(Amps));
    topFlywheelMotor.configure(
        topConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkFlexConfig bottomConfig = new SparkFlexConfig();
    bottomConfig
        .idleMode(IdleMode.kCoast)
        .inverted(BOTTOM_FLYWHEEL_INVERTED)
        .smartCurrentLimit((int) FLYWHEEL_CURRENT_LIMIT.in(Amps));
    bottomFlywheelMotor.configure(
        bottomConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.topFlywheelVelocity = RPM.of(topEncoder.getVelocity());
    inputs.bottomFlywheelVelocity = RPM.of(bottomEncoder.getVelocity());
    inputs.topFlywheelVoltageOut =
        Volts.of(topFlywheelMotor.getAppliedOutput() * topFlywheelMotor.getBusVoltage());
    inputs.bottomFlywheelVoltageOut =
        Volts.of(bottomFlywheelMotor.getAppliedOutput() * bottomFlywheelMotor.getBusVoltage());
    inputs.topFlywheelCurrentOut = Amps.of(topFlywheelMotor.getOutputCurrent());
    inputs.bottomFlywheelCurrentOut = Amps.of(bottomFlywheelMotor.getOutputCurrent());
    inputs.topFlywheelTemp = Celsius.of(topFlywheelMotor.getMotorTemperature());
    inputs.bottomFlywheelTemp = Celsius.of(bottomFlywheelMotor.getMotorTemperature());
    inputs.topFlywheelConnected = !topFlywheelMotor.hasActiveFault();
    inputs.bottomFlywheelConnected = !bottomFlywheelMotor.hasActiveFault();

    topFlywheelDisconnected.set(!inputs.topFlywheelConnected);
    bottomFlywheelDisconnected.set(!inputs.bottomFlywheelConnected);
  }

  @Override
  public void setFlywheelVoltages(Voltage topVoltage, Voltage bottomVoltage) {
    topFlywheelMotor.setVoltage(topVoltage.in(Volts));
    bottomFlywheelMotor.setVoltage(bottomVoltage.in(Volts));
  }
}
