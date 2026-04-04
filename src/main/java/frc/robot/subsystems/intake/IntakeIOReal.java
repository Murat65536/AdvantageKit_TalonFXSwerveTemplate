package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;

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

/** Real hardware IO for the intake roller using a Neo Vortex on a SparkFlex. */
public class IntakeIOReal implements IntakeIO {
  private final SparkFlex rollerMotor = new SparkFlex(ROLLER_MOTOR_ID, MotorType.kBrushless);
  private final RelativeEncoder rollerEncoder = rollerMotor.getEncoder();

  private final Alert rollerDisconnected =
      new Alert("Intake roller SparkFlex disconnected!", AlertType.kError);

  public IntakeIOReal() {
    SparkFlexConfig config = new SparkFlexConfig();
    config
        .idleMode(IdleMode.kBrake)
        .inverted(ROLLER_INVERTED)
        .smartCurrentLimit((int) ROLLER_CURRENT_LIMIT.in(Amps));
    rollerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.rollerVelocity = RPM.of(rollerEncoder.getVelocity());
    inputs.rollerVoltageOut =
        Volts.of(rollerMotor.getAppliedOutput() * rollerMotor.getBusVoltage());
    inputs.rollerCurrentOut = Amps.of(rollerMotor.getOutputCurrent());
    inputs.rollerTemp = Celsius.of(rollerMotor.getMotorTemperature());

    boolean connected = !rollerMotor.hasActiveFault();
    inputs.rollerConnected = connected;
    rollerDisconnected.set(!connected);
  }

  @Override
  public void setRollerVoltage(Voltage voltage) {
    rollerMotor.setVoltage(voltage.in(Volts));
  }
}
