package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.ShooterMath.ShotSolution;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final SysIdRoutine sysId;

  private AngularVelocity targetVelocity = RPM.zero();
  private ShotSolution latestSolution = null;

  public Shooter(ShooterIO io) {
    this.io = io;
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> io.setShooterVoltage(voltage), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IO/Shooter", inputs);
    Logger.recordOutput(
        "Subsystems/Shooter/EstimatedExitVelocityMps",
        ShooterMath.flywheelVelocityToExitVelocity(inputs.velocity).in(MetersPerSecond));
    Logger.recordOutput("Subsystems/Shooter/TargetRpm", targetVelocity.in(RPM));
    Logger.recordOutput("Subsystems/Shooter/AtSetpoint", atSetpoint());
    Logger.recordOutput(
        "Subsystems/Shooter/command",
        getCurrentCommand() == null ? "none" : getCurrentCommand().getName());
  }

  /** Spin up the flywheel at full voltage. Stops when released. */
  public Command shoot() {
    return Commands.startEnd(() -> io.setShooterVoltage(SHOOT_VOLTAGE), this::stop, this)
        .withName("ShooterShoot");
  }

  /** Reverse the flywheel at low voltage (e.g. for unjamming). Stops when released. */
  public Command reverse() {
    return Commands.startEnd(() -> io.setShooterVoltage(REVERSE_VOLTAGE), this::stop, this)
        .withName("ShooterReverse");
  }

  /**
   * Continuously solves the required flywheel speed and hood angle to land in the current target
   * (hub, or corner pass when past the hub), accounting for robot velocity. Requires this and the
   * hood. Does not feed; see {@code ShootCommands}.
   */
  public Command shootAtTarget(
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<ChassisSpeeds> fieldRelativeSpeedsSupplier,
      Hood hood) {
    return Commands.run(
            () -> {
              ShotSolution solution =
                  ShooterMath.calculateShot(
                      robotPoseSupplier.get(), fieldRelativeSpeedsSupplier.get());
              latestSolution = solution;

              AngularVelocity flywheelTarget = solution.flywheelVelocity();
              hood.setTargetAngle(solution.launchAngle());
              setVelocity(flywheelTarget);

              Logger.recordOutput("Subsystems/Shooter/Target", solution.target());
              Logger.recordOutput(
                  "Subsystems/Shooter/TargetExitVelocityMps",
                  solution.exitVelocity().in(MetersPerSecond));
              Logger.recordOutput(
                  "Subsystems/Shooter/TargetFlywheelRpmBase", solution.flywheelVelocity().in(RPM));
              Logger.recordOutput(
                  "Subsystems/Shooter/TargetLaunchAngleDeg", solution.launchAngle().in(Degrees));
              Logger.recordOutput("Subsystems/Shooter/TargetDistanceM", solution.distanceMeters());
              Logger.recordOutput(
                  "Subsystems/Shooter/TargetTofSec", solution.timeOfFlight().in(Seconds));
              Logger.recordOutput("Subsystems/Shooter/TargetInRange", solution.inRange());
            },
            this,
            hood)
        .finallyDo(
            () -> {
              stop();
              latestSolution = null;
            })
        .withName("ShooterShootAtTarget");
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  public void setVelocity(AngularVelocity velocity) {
    targetVelocity = velocity;
    io.setShooterVelocity(velocity);
  }

  public void stop() {
    targetVelocity = RPM.zero();
    io.stop();
  }

  public AngularVelocity getVelocity() {
    return inputs.velocity;
  }

  /** True when a nonzero velocity setpoint is active and the flywheel is within tolerance. */
  public boolean atSetpoint() {
    return targetVelocity.gt(RPM.zero())
        && MathUtil.isNear(
            targetVelocity.in(RPM), inputs.velocity.in(RPM), SHOOTER_AT_SPEED_TOLERANCE.in(RPM));
  }

  /** True when {@link #shootAtTarget} is running and its latest solution is inside the table. */
  public boolean hasShotSolution() {
    return latestSolution != null;
  }

  public boolean isShotInRange() {
    return latestSolution != null && latestSolution.inRange();
  }

  /** Latest shot solution, or null when {@link #shootAtTarget} is not running. */
  public ShotSolution getShotSolution() {
    return latestSolution;
  }
}
