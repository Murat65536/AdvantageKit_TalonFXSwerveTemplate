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
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;

/**
 * Real hardware IO for the hood: two NEO 550s on Spark MAX (right follows left, inverted) and an
 * absolute encoder on the left Spark. Position control is a proportional loop on the RIO that
 * tracks a trapezoid-profiled setpoint, with a stall cutoff.
 */
public class HoodIOReal implements HoodIO {
  private final SparkMax leftMotor = new SparkMax(LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
  private final SparkMax rightMotor = new SparkMax(RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);
  private final AbsoluteEncoder absoluteEncoder = leftMotor.getAbsoluteEncoder();

  private final Alert leftMotorDisconnected =
      new Alert("Hood left Spark MAX disconnected!", AlertType.kError);
  private final Alert rightMotorDisconnected =
      new Alert("Hood right Spark MAX disconnected!", AlertType.kError);

  private final Alert stallAlert =
      new Alert(
          "Hood stalled (pushing into something); output cut until disabled", AlertType.kError);

  private final TrapezoidProfile profile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              HOOD_MAX_VELOCITY_DEG_PER_SEC, HOOD_MAX_ACCEL_DEG_PER_SEC_SQ));
  private final Debouncer stallDebounce =
      new Debouncer(HOOD_STALL_TIME_SECONDS, Debouncer.DebounceType.kRising);

  private Angle targetAngle = HOOD_SOFT_MIN_ANGLE;
  /** Profiled setpoint in hood degrees; the P loop tracks this, not the raw target. */
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  private boolean stallCutoff = false;

  private final Alert notZeroedAlert =
      new Alert(
          "Hood encoder not zeroed: jog the hood to its steep stop and press Pit > Zero Hood",
          AlertType.kError);
  private final Alert zeroFailedAlert =
      new Alert(
          "Zero Hood failed: the Spark did not read 0 with the new offset; old zero kept",
          AlertType.kError);

  /** Loops to wait after writing an offset before trusting the reported position (20 ms each). */
  private static final int ZERO_SETTLE_LOOPS = 5;

  /** The Spark's stored zero offset, read back from it; 0 until zeroed. */
  private double sparkZeroOffsetRot = 0.0;

  private double lastReportedEncoderDeg = 0.0;

  // Zero Hood sequence: try each candidate offset, keep the first the Spark confirms.
  private double[] zeroCandidates = null;
  private int zeroCandidate = 0;
  private int zeroSettleLoops = 0;
  private double offsetBeforeZero = 0.0;

  private boolean jogging = false;
  private double jogVolts = 0.0;

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

    // The zero lives in the Spark's flash (as with REV Hardware Client). The config above never
    // sets it; read back what the Spark has so the log shows whether it survived the boot.
    sparkZeroOffsetRot = leftMotor.configAccessor.absoluteEncoder.getZeroOffset();
    // The zero used to be kept on the roboRIO under this key; drop it so only one zero exists.
    Preferences.remove("Hood/EncoderZeroDeg");
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // Unwrapped so the travel is continuous. Do not wrap the error instead: the travel is exactly
    // half the encoder's period, so a full-travel move (50 -> 76) lands on the wrap boundary and
    // drove the hood the wrong way into the 50 deg stop.
    lastReportedEncoderDeg = absoluteEncoder.getPosition();
    double encoderVelocityDegPerSec = absoluteEncoder.getVelocity();
    Angle angle = reportedEncoderToAngle(lastReportedEncoderDeg);
    boolean zeroed = sparkZeroOffsetRot != 0.0;
    stepZeroSequence();
    double angleDeg = angle.in(Degrees);
    // Encoder increases as the hood retracts, so hood angle velocity is the negative.
    double velocityDegPerSec = -encoderVelocityDegPerSec;
    double currentAmps = leftMotor.getOutputCurrent() + rightMotor.getOutputCurrent();
    double goalDeg = targetAngle.in(Degrees);

    boolean atSetpoint = Math.abs(goalDeg - angleDeg) <= HOOD_ANGLE_TOLERANCE.in(Degrees);
    double controlVolts = 0.0;
    if (zeroCandidates != null) {
      // Mid Zero Hood: the reading may jump between offsets, so hold still and follow it.
      setpoint = new TrapezoidProfile.State(angleDeg, 0.0);
    } else if (DriverStation.isDisabled()) {
      // Start every enable from where the hood actually is, and give a stall another try.
      setpoint = new TrapezoidProfile.State(angleDeg, 0.0);
      stallCutoff = false;
      stallDebounce.calculate(false);
    } else if (jogging) {
      // Open loop for the Pit-tab jog. Track the hood so position control picks up from here,
      // without a jump, once the jog ends.
      setpoint = new TrapezoidProfile.State(angleDeg, 0.0);
      controlVolts = jogAllowed(jogVolts, angleDeg, zeroed) ? jogVolts : 0.0;
    } else {
      setpoint = profile.calculate(0.02, setpoint, new TrapezoidProfile.State(goalDeg, 0.0));
      boolean profileDone = setpoint.position == goalDeg && setpoint.velocity == 0.0;
      if (!(atSetpoint && profileDone)) {
        // Volts push the encoder up and hood angle = MAX - encoder, hence angle - setpoint.
        controlVolts =
            MathUtil.clamp(
                (angleDeg - setpoint.position) * HOOD_KP_VOLTS_PER_DEG,
                -MAX_CONTROL_VOLTAGE.in(Volts),
                MAX_CONTROL_VOLTAGE.in(Volts));
      }
    }
    if (DriverStation.isEnabled()) {
      boolean stalling =
          currentAmps > HOOD_STALL_CURRENT.in(Amps)
              && Math.abs(velocityDegPerSec) < HOOD_STALL_VELOCITY_DEG_PER_SEC;
      if (stallDebounce.calculate(stalling)) {
        stallCutoff = true;
      }
    }
    leftMotor.setVoltage(stallCutoff ? 0.0 : controlVolts);
    stallAlert.set(stallCutoff);

    inputs.angle = angle;
    inputs.velocity = DegreesPerSecond.of(velocityDegPerSec);
    inputs.appliedVoltage = Volts.of(leftMotor.getAppliedOutput() * leftMotor.getBusVoltage());
    inputs.current = Amps.of(currentAmps);
    inputs.stallCutoff = stallCutoff;
    inputs.reportedEncoderDeg = lastReportedEncoderDeg;
    inputs.sparkZeroOffsetRot = sparkZeroOffsetRot;
    inputs.zeroed = zeroed;
    inputs.zeroInProgress = zeroCandidates != null;
    inputs.jogging = jogging;
    notZeroedAlert.set(!zeroed);
    inputs.temp =
        Celsius.of(Math.max(leftMotor.getMotorTemperature(), rightMotor.getMotorTemperature()));
    inputs.leftMotorConnected = leftMotor.getLastError() == REVLibError.kOk;
    inputs.rightMotorConnected = rightMotor.getLastError() == REVLibError.kOk;
    inputs.encoderConnected = !leftMotor.getFaults().sensor;
    inputs.atSetpoint = atSetpoint;

    leftMotorDisconnected.set(!inputs.leftMotorConnected);
    rightMotorDisconnected.set(!inputs.rightMotorConnected);
  }

  /**
   * Starts Zero Hood: writes the zero offset to the Spark so the current position (the steep stop)
   * reports 0, checks it took, then saves it to the Spark's flash. See {@link #stepZeroSequence}.
   */
  @Override
  public void zeroAtSteepStop() {
    offsetBeforeZero = sparkZeroOffsetRot;
    zeroCandidates =
        sparkZeroOffsetCandidates(
            lastReportedEncoderDeg, offsetBeforeZero, ABSOLUTE_ENCODER_INVERTED);
    zeroCandidate = 0;
    zeroFailedAlert.set(false);
    tryZeroCandidate();
  }

  private void tryZeroCandidate() {
    writeZeroOffset(zeroCandidates[zeroCandidate], false);
    zeroSettleLoops = ZERO_SETTLE_LOOPS;
  }

  /**
   * Runs once per loop while zeroing. Waits for the Spark to report with the new offset, keeps it
   * (and saves it to flash) if the hood now reads as the steep stop, otherwise tries the next
   * candidate, and puts the old offset back if none work.
   */
  private void stepZeroSequence() {
    if (zeroCandidates == null || --zeroSettleLoops > 0) {
      return;
    }
    if (readsAsSteepStop(lastReportedEncoderDeg)) {
      writeZeroOffset(zeroCandidates[zeroCandidate], true);
      zeroCandidates = null;
      // The hood is now at MAX by definition; restart the profile there so it doesn't slew from
      // the pre-zero reading back to the target.
      setpoint = new TrapezoidProfile.State(MAX_HOOD_ANGLE.in(Degrees), 0.0);
      return;
    }
    zeroCandidate++;
    if (zeroCandidate < zeroCandidates.length) {
      tryZeroCandidate();
      return;
    }
    writeZeroOffset(offsetBeforeZero, false);
    zeroCandidates = null;
    zeroFailedAlert.set(true);
    // The Hood asked for MAX expecting the new zero; hold where the hood really is instead.
    targetAngle =
        Degrees.of(clampTargetDeg(reportedEncoderToAngle(lastReportedEncoderDeg).in(Degrees)));
  }

  /**
   * Sets the Spark's absolute encoder zero offset, touching nothing else. With {@code persist} the
   * Spark also saves its settings to flash, so the zero survives power cycles.
   */
  private void writeZeroOffset(double offsetRot, boolean persist) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.absoluteEncoder.zeroOffset(offsetRot);
    leftMotor.configure(
        config,
        ResetMode.kNoResetSafeParameters,
        persist ? PersistMode.kPersistParameters : PersistMode.kNoPersistParameters);
    sparkZeroOffsetRot = leftMotor.configAccessor.absoluteEncoder.getZeroOffset();
  }

  @Override
  public void startJog(Voltage volts) {
    jogVolts = volts.in(Volts);
    jogging = true;
  }

  @Override
  public void stopJog() {
    jogging = false;
    jogVolts = 0.0;
  }

  @Override
  public void setTargetAngle(Angle angle) {
    targetAngle = Degrees.of(clampTargetDeg(angle.in(Degrees)));
  }
}
