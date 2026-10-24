package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.subsystems.hood.HoodConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private Angle targetAngle = HOOD_SOFT_MIN_ANGLE;
  /** Operator trim added to every requested angle, so shots can be nudged live. */
  private Angle trim = Degrees.zero();

  public Hood(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IO/Hood", inputs);
    if (DriverStation.isDisabled()) {
      // Hold wherever the hood is, so enabling never moves it on its own.
      setTargetAngleUntrimmed(inputs.angle);
    }
    Logger.recordOutput("Subsystems/Hood/ComponentPose", getComponentPose());
    Logger.recordOutput("Subsystems/Hood/TargetAngleDeg", targetAngle.in(Degrees));
    Logger.recordOutput("Subsystems/Hood/TrimDeg", trim.in(Degrees));
    Logger.recordOutput(
        "Subsystems/Hood/command",
        getCurrentCommand() == null ? "none" : getCurrentCommand().getName());
  }

  /**
   * Sets the hood target (trim is applied on top), clamped to the soft minimum and the steep stop.
   */
  public void setTargetAngle(Angle angle) {
    targetAngle = Degrees.of(clampTargetDeg(angle.plus(trim).in(Degrees)));
    io.setTargetAngle(targetAngle);
  }

  /** Adjusts the persistent trim offset. Does not require the subsystem. */
  public Command adjustTrim(Angle delta) {
    return Commands.runOnce(
            () ->
                trim =
                    Degrees.of(
                        MathUtil.clamp(
                            trim.plus(delta).in(Degrees),
                            -MAX_TRIM.in(Degrees),
                            MAX_TRIM.in(Degrees))))
        .ignoringDisable(true)
        .withName("HoodAdjustTrim");
  }

  /** Clears the trim offset. Does not require the subsystem. */
  public Command resetTrim() {
    return Commands.runOnce(() -> trim = Degrees.zero())
        .ignoringDisable(true)
        .withName("HoodResetTrim");
  }

  /**
   * Zero the hood encoder: with the hood against its steep (fully retracted) hard stop, run this.
   * That position then reads {@link HoodConstants#MAX_HOOD_ANGLE}. The zero is the Spark's own
   * absolute encoder zero offset, saved to its flash: the same setting REV Hardware Client writes,
   * so either can be used. Works enabled or disabled: the target moves to the new reading in the
   * same step, so the hood holds still instead of chasing its old target. Refuses to run on the
   * field.
   */
  public Command zeroAtSteepStop() {
    return Commands.runOnce(
            () -> {
              io.zeroAtSteepStop();
              setTargetAngleUntrimmed(MAX_HOOD_ANGLE);
            })
        .onlyIf(() -> !DriverStation.isFMSAttached())
        .ignoringDisable(true)
        .withName("HoodZeroAtSteepStop");
  }

  /** One short, slow nudge toward the steep stop (see {@link #jog}). */
  public Command jogSteeper() {
    return jog(HOOD_JOG_VOLTAGE.unaryMinus()).withName("HoodJogSteeper");
  }

  /** One short, slow nudge toward the shallow stop (see {@link #jog}). */
  public Command jogShallower() {
    return jog(HOOD_JOG_VOLTAGE).withName("HoodJogShallower");
  }

  /**
   * Drive the hood open loop at {@code volts} for {@link HoodConstants#HOOD_JOG_SECONDS}, for
   * zeroing from the Pit tab. Works before the hood is zeroed; the stall cutoff still applies.
   * Needs the robot enabled, and refuses to run on the field. Afterwards the hood holds where it
   * stopped.
   */
  private Command jog(Voltage volts) {
    return Commands.startEnd(
            () -> io.startJog(volts),
            () -> {
              io.stopJog();
              setTargetAngleUntrimmed(inputs.angle);
            },
            this)
        .withTimeout(HOOD_JOG_SECONDS)
        .onlyIf(() -> !DriverStation.isFMSAttached());
  }

  /** Sets the hood target ignoring the driver's trim, so a check tests the real limits. */
  public void setTargetAngleUntrimmed(Angle angle) {
    targetAngle = Degrees.of(clampTargetDeg(angle.in(Degrees)));
    io.setTargetAngle(targetAngle);
  }

  /** Both motors and the absolute encoder. */
  public boolean isConnected() {
    return inputs.leftMotorConnected && inputs.rightMotorConnected && inputs.encoderConnected;
  }

  public Angle getAngle() {
    return inputs.angle;
  }

  public Angle getTargetAngle() {
    return targetAngle;
  }

  public boolean atSetpoint() {
    return inputs.atSetpoint;
  }

  /** Returns the current hood pose in field coordinates for simulation visualization. */
  public Pose3d getPose(Pose2d robotPose) {
    Pose3d componentPose = getComponentPose();
    return new Pose3d(robotPose)
        .transformBy(new Transform3d(componentPose.getTranslation(), componentPose.getRotation()));
  }

  private Pose3d getComponentPose() {
    double relativeHoodAngleRad = getAngle().minus(MIN_HOOD_ANGLE).in(Radians);
    return new Pose3d(HOOD_PIVOT_TRANSLATION, new Rotation3d(0.0, relativeHoodAngleRad, 0.0));
  }
}
