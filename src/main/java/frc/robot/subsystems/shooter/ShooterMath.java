package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import java.util.Optional;

public final class ShooterMath {
  private static final InterpolatingTreeMap<Double, Rotation2d> launchAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap exitVelocityMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();
  private static final double minDistanceMeters;
  private static final double maxDistanceMeters;

  private ShooterMath() {}

  static {
    // Tuned map seeds inspired by LaunchCalculator-style interpolation.
    launchAngleMap.put(1.0, Rotation2d.fromDegrees(50.0));
    launchAngleMap.put(1.5, Rotation2d.fromDegrees(48.0));
    launchAngleMap.put(2.0, Rotation2d.fromDegrees(46.0));
    launchAngleMap.put(2.5, Rotation2d.fromDegrees(44.0));
    launchAngleMap.put(3.0, Rotation2d.fromDegrees(42.0));
    launchAngleMap.put(3.5, Rotation2d.fromDegrees(40.0));
    launchAngleMap.put(4.0, Rotation2d.fromDegrees(38.0));
    launchAngleMap.put(4.5, Rotation2d.fromDegrees(36.0));

    exitVelocityMap.put(1.0, 10.0);
    exitVelocityMap.put(1.5, 11.5);
    exitVelocityMap.put(2.0, 12.8);
    exitVelocityMap.put(2.5, 14.0);
    exitVelocityMap.put(3.0, 15.3);
    exitVelocityMap.put(3.5, 16.6);
    exitVelocityMap.put(4.0, 17.8);
    exitVelocityMap.put(4.5, 19.0);

    timeOfFlightMap.put(1.0, 0.55);
    timeOfFlightMap.put(1.5, 0.65);
    timeOfFlightMap.put(2.0, 0.78);
    timeOfFlightMap.put(2.5, 0.92);
    timeOfFlightMap.put(3.0, 1.05);
    timeOfFlightMap.put(3.5, 1.18);
    timeOfFlightMap.put(4.0, 1.30);
    timeOfFlightMap.put(4.5, 1.45);

    minDistanceMeters = 1.0;
    maxDistanceMeters = 4.5;
  }

  public record ShotSolution(
      LinearVelocity exitVelocity, Angle launchAngle, Time timeOfFlight, double distanceMeters) {}

  /**
   * Map-based launch model: apply lookahead from chassis velocity, then interpolate speed and angle
   * by distance.
   */
  public static Optional<ShotSolution> calculateShotForHub(
      Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
    Translation2d launchTranslation =
        robotPose
            .getTranslation()
            .plus(SHOOTER_POSITION_ON_ROBOT.rotateBy(robotPose.getRotation()));

    Translation2d lookaheadLaunchTranslation = launchTranslation;
    double lookaheadDistanceMeters = HUB_TRANSLATION.getDistance(lookaheadLaunchTranslation);
    for (int i = 0; i < 8; i++) {
      double tofSeconds = timeOfFlightMap.get(lookaheadDistanceMeters);
      Translation2d velocityOffset =
          new Translation2d(
              fieldRelativeSpeeds.vxMetersPerSecond * tofSeconds,
              fieldRelativeSpeeds.vyMetersPerSecond * tofSeconds);
      lookaheadLaunchTranslation = launchTranslation.plus(velocityOffset);
      lookaheadDistanceMeters = HUB_TRANSLATION.getDistance(lookaheadLaunchTranslation);
    }

    if (lookaheadDistanceMeters < minDistanceMeters
        || lookaheadDistanceMeters > maxDistanceMeters) {
      return Optional.empty();
    }

    double speedMetersPerSecond =
        MathUtil.clamp(
            exitVelocityMap.get(lookaheadDistanceMeters),
            MIN_DYNAMIC_EXIT_VELOCITY.in(MetersPerSecond),
            MAX_DYNAMIC_EXIT_VELOCITY.in(MetersPerSecond));
    Angle launchAngle =
        Degrees.of(
            MathUtil.clamp(
                launchAngleMap.get(lookaheadDistanceMeters).getDegrees(),
                MIN_DYNAMIC_SHOOTER_ANGLE.in(Degrees),
                MAX_DYNAMIC_SHOOTER_ANGLE.in(Degrees)));

    return Optional.of(
        new ShotSolution(
            MetersPerSecond.of(speedMetersPerSecond),
            launchAngle,
            Seconds.of(timeOfFlightMap.get(lookaheadDistanceMeters)),
            lookaheadDistanceMeters));
  }

  public static AngularVelocity exitVelocityToFlywheelVelocity(LinearVelocity exitVelocity) {
    double targetRadPerSecond =
        MathUtil.clamp(
            exitVelocity.in(MetersPerSecond) / EXIT_VELOCITY_PER_FLYWHEEL_RAD_PER_SEC,
            0.0,
            MAX_FLYWHEEL_VELOCITY.in(RadiansPerSecond));
    return RadiansPerSecond.of(targetRadPerSecond);
  }

  public static LinearVelocity flywheelVelocityToExitVelocity(AngularVelocity flywheelVelocity) {
    double exitMetersPerSecond =
        Math.max(
            0.0, flywheelVelocity.in(RadiansPerSecond) * EXIT_VELOCITY_PER_FLYWHEEL_RAD_PER_SEC);
    return MetersPerSecond.of(exitMetersPerSecond);
  }
}
