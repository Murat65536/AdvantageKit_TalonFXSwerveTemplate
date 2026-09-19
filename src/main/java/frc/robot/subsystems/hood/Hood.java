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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private Angle targetAngle = MIN_HOOD_ANGLE;
  /** Operator trim added to every requested angle, so shots can be nudged live. */
  private Angle trim = Degrees.zero();

  public Hood(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IO/Hood", inputs);
    Logger.recordOutput("Subsystems/Hood/ComponentPose", getComponentPose());
    Logger.recordOutput("Subsystems/Hood/TargetAngleDeg", targetAngle.in(Degrees));
    Logger.recordOutput("Subsystems/Hood/TrimDeg", trim.in(Degrees));
    Logger.recordOutput(
        "Subsystems/Hood/command",
        getCurrentCommand() == null ? "none" : getCurrentCommand().getName());
  }

  /** Sets the hood target (trim is applied on top), clamped to the mechanical limits. */
  public void setTargetAngle(Angle angle) {
    targetAngle =
        Degrees.of(
            MathUtil.clamp(
                angle.plus(trim).in(Degrees),
                MIN_HOOD_ANGLE.in(Degrees),
                MAX_HOOD_ANGLE.in(Degrees)));
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
