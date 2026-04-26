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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private Angle targetAngle = MIN_HOOD_ANGLE;

  public Hood(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IO/Hood", inputs);
    Pose3d componentPose = getComponentPose();
    Logger.recordOutput("Subsystems/Hood/ComponentPose", componentPose);
    Logger.recordOutput(
        "Subsystems/Hood/command",
        getCurrentCommand() == null ? "none" : getCurrentCommand().getName());
  }

  public void setTargetAngle(Angle angle) {
    targetAngle =
        Degrees.of(
            MathUtil.clamp(
                angle.in(Degrees), MIN_HOOD_ANGLE.in(Degrees), MAX_HOOD_ANGLE.in(Degrees)));
    io.setTargetAngle(targetAngle);
  }

  /** Adjusts hood target angle by a delta, clamped to min/max limits. */
  public void adjustTargetAngle(Angle delta) {
    setTargetAngle(targetAngle.plus(delta));
  }

  public Angle getAngle() {
    return inputs.angle;
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
