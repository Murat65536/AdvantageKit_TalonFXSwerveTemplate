package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IO/Shooter", inputs);
    Logger.recordOutput(
        "Subsystems/Shooter/command",
        getCurrentCommand() == null ? "none" : getCurrentCommand().getName());
  }

  /** Spin up the flywheel at full voltage. Stops when released. */
  public Command shoot() {
    return Commands.startEnd(
        () -> io.setShooterVoltage(SHOOT_VOLTAGE), () -> io.setShooterVoltage(Volts.zero()), this);
  }

  /** Reverse the flywheel at low voltage (e.g. for unjamming). Stops when released. */
  public Command reverse() {
    return Commands.startEnd(
        () -> io.setShooterVoltage(REVERSE_VOLTAGE),
        () -> io.setShooterVoltage(Volts.zero()),
        this);
  }

  /** Continuously solves required shot speed and launch angle to land in the hub. */
  public Command shootAtHub(
      Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> fieldRelativeSpeedsSupplier) {
    return Commands.run(
            () -> {
              var shotSolution =
                  ShooterMath.calculateShotForHub(
                      robotPoseSupplier.get(), fieldRelativeSpeedsSupplier.get());
              if (shotSolution.isEmpty()) {
                io.setShooterVoltage(Volts.zero());
                Logger.recordOutput("Subsystems/Shooter/HubTargetExitVelocityMps", 0.0);
                Logger.recordOutput("Subsystems/Shooter/HubTargetLaunchAngleDeg", 0.0);
                Logger.recordOutput("Subsystems/Shooter/HubTargetDistanceM", 0.0);
                Logger.recordOutput("Subsystems/Shooter/HubTargetTofSec", 0.0);
                return;
              }

              io.setShooterLaunchAngle(shotSolution.get().launchAngle());
              io.setShooterVelocity(
                  ShooterMath.exitVelocityToFlywheelVelocity(shotSolution.get().exitVelocity()));
              Logger.recordOutput(
                  "Subsystems/Shooter/HubTargetExitVelocityMps",
                  shotSolution.get().exitVelocity().in(MetersPerSecond));
              Logger.recordOutput(
                  "Subsystems/Shooter/HubTargetLaunchAngleDeg",
                  shotSolution.get().launchAngle().in(Degrees));
              Logger.recordOutput(
                  "Subsystems/Shooter/HubTargetDistanceM", shotSolution.get().distanceMeters());
              Logger.recordOutput(
                  "Subsystems/Shooter/HubTargetTofSec",
                  shotSolution.get().timeOfFlight().in(Seconds));
            },
            this)
        .finallyDo(() -> io.setShooterVoltage(Volts.zero()));
  }
}
