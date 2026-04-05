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
  private final DCMotorSim topFlywheelSim;
  private final DCMotorSim bottomFlywheelSim;
  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<ChassisSpeeds> fieldRelativeSpeedsSupplier;
  private final BooleanSupplier consumeGamePiece;
  private Voltage topAppliedVoltage = Volts.zero();
  private Voltage bottomAppliedVoltage = Volts.zero();
  private double lastShotTimeSeconds = Double.NEGATIVE_INFINITY;

  public ShooterIOSim(
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<ChassisSpeeds> fieldRelativeSpeedsSupplier,
      BooleanSupplier consumeGamePiece) {
    topFlywheelSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getNeoVortex(1), FLYWHEEL_MOI, FLYWHEEL_GEAR_RATIO),
            DCMotor.getNeoVortex(1));
    bottomFlywheelSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getNeoVortex(1), FLYWHEEL_MOI, FLYWHEEL_GEAR_RATIO),
            DCMotor.getNeoVortex(1));
    this.robotPoseSupplier = robotPoseSupplier;
    this.fieldRelativeSpeedsSupplier = fieldRelativeSpeedsSupplier;
    this.consumeGamePiece = consumeGamePiece;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    topFlywheelSim.setInputVoltage(topAppliedVoltage.in(Volts));
    bottomFlywheelSim.setInputVoltage(bottomAppliedVoltage.in(Volts));
    topFlywheelSim.update(0.02);
    bottomFlywheelSim.update(0.02);

    inputs.topFlywheelVelocity = RadiansPerSecond.of(topFlywheelSim.getAngularVelocityRadPerSec());
    inputs.bottomFlywheelVelocity =
        RadiansPerSecond.of(bottomFlywheelSim.getAngularVelocityRadPerSec());
    inputs.topFlywheelVoltageOut = topAppliedVoltage;
    inputs.bottomFlywheelVoltageOut = bottomAppliedVoltage;
    inputs.topFlywheelCurrentOut = Amps.of(topFlywheelSim.getCurrentDrawAmps());
    inputs.bottomFlywheelCurrentOut = Amps.of(bottomFlywheelSim.getCurrentDrawAmps());
    inputs.topFlywheelConnected = true;
    inputs.bottomFlywheelConnected = true;

    maybeLaunchProjectile(inputs.topFlywheelVelocity, inputs.bottomFlywheelVelocity);
  }

  @Override
  public void setFlywheelVoltages(Voltage topVoltage, Voltage bottomVoltage) {
    topAppliedVoltage = topVoltage;
    bottomAppliedVoltage = bottomVoltage;
  }

  private void maybeLaunchProjectile(
      AngularVelocity topFlywheelVelocity, AngularVelocity bottomFlywheelVelocity) {
    if (topAppliedVoltage.lt(Volts.of(6))
        || bottomAppliedVoltage.lt(Volts.of(6))
        || topFlywheelVelocity.lt(SHOOTER_READY_VELOCITY)
        || bottomFlywheelVelocity.lt(SHOOTER_READY_VELOCITY)) {
      return;
    }

    double now = Logger.getTimestamp() / 1.0e6;
    if (now - lastShotTimeSeconds < SHOOTER_SHOT_PERIOD.in(Seconds)) {
      return;
    }

    Pose2d robotPose = robotPoseSupplier.get();
    List<Pose3d> topTrajectory = new ArrayList<>();
    List<Pose3d> bottomTrajectory = new ArrayList<>();
    boolean launchedProjectile = false;

    if (consumeGamePiece.getAsBoolean()) {
      launchProjectile(robotPose, TOP_SHOOTER_POSITION_ON_ROBOT, "Top", topTrajectory);
      launchedProjectile = true;
    } else {
      Logger.recordOutput("FieldSimulation/TopShooterTrajectory", new Pose3d[] {});
    }

    if (consumeGamePiece.getAsBoolean()) {
      launchProjectile(robotPose, BOTTOM_SHOOTER_POSITION_ON_ROBOT, "Bottom", bottomTrajectory);
      launchedProjectile = true;
    } else {
      Logger.recordOutput("FieldSimulation/BottomShooterTrajectory", new Pose3d[] {});
    }

    Logger.recordOutput(
        "FieldSimulation/ShooterTrajectory",
        combineTrajectories(topTrajectory, bottomTrajectory).toArray(Pose3d[]::new));

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

  private List<Pose3d> combineTrajectories(
      List<Pose3d> topTrajectory, List<Pose3d> bottomTrajectory) {
    List<Pose3d> combinedTrajectory =
        new ArrayList<>(topTrajectory.size() + bottomTrajectory.size());
    combinedTrajectory.addAll(topTrajectory);
    combinedTrajectory.addAll(bottomTrajectory);
    return combinedTrajectory;
  }
}
