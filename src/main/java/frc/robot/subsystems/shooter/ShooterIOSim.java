package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

/** Simulated IO for the shooter flywheel and projectile launching using MapleSim. */
public class ShooterIOSim implements ShooterIO {
  private final DCMotorSim shooterSim;
  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<ChassisSpeeds> fieldRelativeSpeedsSupplier;
  private final BooleanSupplier consumeGamePiece;
  private Voltage appliedVoltage = Volts.zero();
  private double lastShotTimeSeconds = Double.NEGATIVE_INFINITY;

  public ShooterIOSim(
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<ChassisSpeeds> fieldRelativeSpeedsSupplier,
      BooleanSupplier consumeGamePiece) {
    shooterSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getNeoVortex(4), FLYWHEEL_MOI, FLYWHEEL_GEAR_RATIO),
            DCMotor.getNeoVortex(4));
    this.robotPoseSupplier = robotPoseSupplier;
    this.fieldRelativeSpeedsSupplier = fieldRelativeSpeedsSupplier;
    this.consumeGamePiece = consumeGamePiece;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
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
    appliedVoltage = voltage;
  }

  private void maybeLaunchProjectile(AngularVelocity velocity) {
    if (appliedVoltage.lt(Volts.of(6)) || velocity.lt(SHOOTER_READY_VELOCITY)) {
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
      launchProjectile(robotPose, SHOOTER_POSITION_ON_ROBOT, "Shooter", trajectory);
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
      Translation2d shooterPositionOnRobot,
      String trajectoryKeySuffix,
      List<Pose3d> trajectoryBuffer) {
    RebuiltFuelOnFly shot =
        new RebuiltFuelOnFly(
            robotPose.getTranslation(),
            shooterPositionOnRobot,
            fieldRelativeSpeedsSupplier.get(),
            robotPose.getRotation(),
            SHOOTER_HEIGHT,
            SHOOTER_EXIT_VELOCITY,
            SHOOTER_ANGLE);
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
