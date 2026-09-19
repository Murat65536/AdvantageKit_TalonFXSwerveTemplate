package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

/**
 * Real hardware IO for the shooter: four Neo Vortex motors on Spark MAX controllers. Left-top is
 * the leader; left-bottom follows it, and both right motors follow inverted. Velocity closed-loop
 * runs onboard the leader Spark.
 */
public class ShooterIOReal implements ShooterIO {
  private final SparkMax leftTop = new SparkMax(LEFT_TOP_MOTOR_CAN_ID, MotorType.kBrushless);
  private final SparkMax leftBottom = new SparkMax(LEFT_BOTTOM_MOTOR_CAN_ID, MotorType.kBrushless);
  private final SparkMax rightTop = new SparkMax(RIGHT_TOP_MOTOR_CAN_ID, MotorType.kBrushless);
  private final SparkMax rightBottom =
      new SparkMax(RIGHT_BOTTOM_MOTOR_CAN_ID, MotorType.kBrushless);
  private final SparkMax[] allMotors = {leftTop, leftBottom, rightTop, rightBottom};
  private final RelativeEncoder encoder = leftTop.getEncoder();
  private final SparkClosedLoopController controller = leftTop.getClosedLoopController();

  private final Alert shooterDisconnected =
      new Alert("Shooter Spark MAX disconnected!", AlertType.kError);
  private AngularVelocity velocitySetpoint = RPM.zero();

  public ShooterIOReal() {
    SparkMaxConfig baseConfig = new SparkMaxConfig();
    baseConfig
        .idleMode(IdleMode.kCoast)
        .inverted(LEADER_INVERTED)
        .smartCurrentLimit((int) CURRENT_LIMIT.in(Amps));
    baseConfig
        .closedLoop
        .pid(VELOCITY_KP, VELOCITY_KI, VELOCITY_KD)
        .feedForward
        .sva(VELOCITY_KS, VELOCITY_KV, VELOCITY_KA);
    baseConfig.encoder.quadratureAverageDepth(5).quadratureMeasurementPeriod(10);
    leftTop.configure(baseConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    leftFollowerConfig.apply(baseConfig).follow(leftTop, false);
    leftBottom.configure(
        leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();
    rightFollowerConfig.apply(baseConfig).follow(leftTop, true);
    rightTop.configure(
        rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    rightBottom.configure(
        rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    double totalCurrent = 0.0;
    double maxTemp = 0.0;
    boolean allConnected = true;
    for (SparkMax motor : allMotors) {
      totalCurrent += motor.getOutputCurrent();
      maxTemp = Math.max(maxTemp, motor.getMotorTemperature());
      allConnected &= motor.getLastError() == REVLibError.kOk;
    }

    inputs.position = Rotations.of(encoder.getPosition());
    inputs.velocity = RPM.of(encoder.getVelocity());
    inputs.velocitySetpoint = velocitySetpoint;
    inputs.voltageOut = Volts.of(leftTop.getAppliedOutput() * leftTop.getBusVoltage());
    inputs.currentOut = Amps.of(totalCurrent);
    inputs.temp = Celsius.of(maxTemp);
    inputs.connected = allConnected;

    shooterDisconnected.set(!inputs.connected);
  }

  @Override
  public void setShooterVoltage(Voltage voltage) {
    velocitySetpoint = RPM.zero();
    leftTop.setVoltage(voltage.in(Volts));
  }

  @Override
  public void setShooterVelocity(AngularVelocity velocity) {
    velocitySetpoint = velocity;
    controller.setSetpoint(velocity.in(RPM), ControlType.kVelocity);
  }

  @Override
  public void stop() {
    velocitySetpoint = RPM.zero();
    leftTop.stopMotor();
  }
}
