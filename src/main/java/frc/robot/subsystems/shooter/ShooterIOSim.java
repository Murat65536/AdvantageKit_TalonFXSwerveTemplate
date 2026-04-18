package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

/** Simulated IO for the shooter flywheel and projectile launching using MapleSim. */
public class ShooterIOSim implements ShooterIO {
  private final FlywheelSim shooterSim;
  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<ChassisSpeeds> fieldRelativeSpeedsSupplier;
  private final Supplier<Angle> launchAngleSupplier;
  private final BooleanSupplier consumeGamePiece;
  private final double simMaxFlywheelSpeedRadPerSec = DCMotor.getNeoVortex(4).freeSpeedRadPerSec;
  private Voltage appliedVoltage = Volts.zero();
  private AngularVelocity velocitySetpoint = RPM.zero();
  private LinearVelocity requestedExitVelocity = SHOOTER_EXIT_VELOCITY;
  private double lastShotTimeSeconds = Double.NEGATIVE_INFINITY;

  public ShooterIOSim(
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<ChassisSpeeds> fieldRelativeSpeedsSupplier,
      Supplier<Angle> launchAngleSupplier,
      BooleanSupplier consumeGamePiece) {
    shooterSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNeoVortex(4), FLYWHEEL_MOI, FLYWHEEL_GEAR_RATIO),
            DCMotor.getNeoVortex(4));
    this.robotPoseSupplier = robotPoseSupplier;
    this.fieldRelativeSpeedsSupplier = fieldRelativeSpeedsSupplier;
    this.launchAngleSupplier = launchAngleSupplier;
    this.consumeGamePiece = consumeGamePiece;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    double targetRadPerSec = velocitySetpoint.in(RadiansPerSecond);
    double measuredRadPerSec = shooterSim.getAngularVelocityRadPerSec();
    double ffVolts = 12.0 * targetRadPerSec / simMaxFlywheelSpeedRadPerSec;
    double feedbackVolts =
        (targetRadPerSec - measuredRadPerSec) * SHOOTER_SIM_VELOCITY_KP_VOLTS_PER_RAD_PER_SEC;
    appliedVoltage = Volts.of(MathUtil.clamp(ffVolts + feedbackVolts, -12.0, 12.0));

    shooterSim.setInputVoltage(appliedVoltage.in(Volts));
    shooterSim.update(0.02);

    inputs.velocity = RadiansPerSecond.of(shooterSim.getAngularVelocityRadPerSec());
    inputs.voltageOut = appliedVoltage;
    inputs.currentOut = Amps.of(shooterSim.getCurrentDrawAmps());
    inputs.connected = true;

    maybeLaunchProjectile(inputs.velocity);
  }

  @Override
  public void setShooterVoltage(Voltage voltage) {
    double targetRadPerSec =
        MathUtil.clamp(voltage.in(Volts), -12.0, 12.0)
            / 12.0
            * MAX_FLYWHEEL_VELOCITY.in(RadiansPerSecond);
    velocitySetpoint = RadiansPerSecond.of(targetRadPerSec);
    requestedExitVelocity = ShooterMath.flywheelVelocityToExitVelocity(velocitySetpoint);
  }

  @Override
  public void setShooterVelocity(AngularVelocity velocity) {
    velocitySetpoint = velocity;
    requestedExitVelocity = ShooterMath.flywheelVelocityToExitVelocity(velocity);
  }

  private void maybeLaunchProjectile(AngularVelocity velocity) {
    if (velocitySetpoint.lte(RPM.zero())) {
      return;
    }

    if (velocity.lt(velocitySetpoint.minus(SHOOTER_AT_SPEED_TOLERANCE))) {
      return;
    }

    double now = Logger.getTimestamp() / 1.0e6;
    if (now - lastShotTimeSeconds < SHOOTER_SHOT_PERIOD.in(Seconds)) {
      return;
    }

    Pose2d robotPose = robotPoseSupplier.get();
    List<Pose3d> trajectory = new ArrayList<>();
    boolean launchedProjectile = false;

    if (consumeGamePiece.getAsBoolean()) {
      launchProjectile(
          robotPose, BALL_EXIT_TRANSLATION, "Shooter", requestedExitVelocity, trajectory);
      launchedProjectile = true;
    } else {
      Logger.recordOutput("FieldSimulation/ShooterTrajectory", new Pose3d[] {});
    }

    if (launchedProjectile) {
      lastShotTimeSeconds = now;
    }
  }

  private void launchProjectile(
      Pose2d robotPose,
      Translation3d ballExitTranslation,
      String trajectoryKeySuffix,
      LinearVelocity exitVelocity,
      List<Pose3d> trajectoryBuffer) {
    RebuiltFuelOnFly shot =
        new RebuiltFuelOnFly(
            robotPose.getTranslation(),
            ballExitTranslation.toTranslation2d(),
            fieldRelativeSpeedsSupplier.get(),
            robotPose.getRotation().plus(new Rotation2d(Math.PI)),
            Meters.of(ballExitTranslation.getZ()),
            exitVelocity,
            launchAngleSupplier.get());
    shot.withProjectileTrajectoryDisplayCallBack(
        trajectory -> {
          trajectoryBuffer.clear();
          trajectoryBuffer.addAll(trajectory);
          Logger.recordOutput(
              "FieldSimulation/" + trajectoryKeySuffix + "ShooterTrajectory",
              trajectory.toArray(Pose3d[]::new));
        });
    SimulatedArena.getInstance().addGamePieceProjectile(shot);
  }
}
