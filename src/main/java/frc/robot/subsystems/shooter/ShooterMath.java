package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import java.util.Optional;

public final class ShooterMath {
  private static final InterpolatingDoubleTreeMap exitVelocityMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();
  private static final Angle fixedLaunchAngle = Degrees.of(59.0);
  private static final double minDistanceMeters;
  private static final double maxDistanceMeters;

  private ShooterMath() {}

  static {
    exitVelocityMap.put(2.286, rpmToVelocity(2650));
    exitVelocityMap.put(2.540, rpmToVelocity(2700));
    exitVelocityMap.put(2.794, rpmToVelocity(2800));
    exitVelocityMap.put(3.048, rpmToVelocity(2900));
    exitVelocityMap.put(3.302, rpmToVelocity(3000));
    exitVelocityMap.put(3.556, rpmToVelocity(3100));
    exitVelocityMap.put(3.810, rpmToVelocity(3250));
    exitVelocityMap.put(4.064, rpmToVelocity(3300));
    exitVelocityMap.put(4.318, rpmToVelocity(3400));
    exitVelocityMap.put(4.572, rpmToVelocity(3500));
    exitVelocityMap.put(4.826, rpmToVelocity(3650));
    exitVelocityMap.put(5.080, rpmToVelocity(3750));
    exitVelocityMap.put(5.334, rpmToVelocity(3800));

    timeOfFlightMap.put(1.0, 0.55);
    timeOfFlightMap.put(1.5, 0.65);
    timeOfFlightMap.put(2.0, 0.78);
    timeOfFlightMap.put(2.5, 0.92);
    timeOfFlightMap.put(3.0, 1.05);
    timeOfFlightMap.put(3.5, 1.18);
    timeOfFlightMap.put(4.0, 1.30);
    timeOfFlightMap.put(4.5, 1.45);

    minDistanceMeters = 2.286;
    maxDistanceMeters = 5.334;
  }

  public record ShotSolution(
      LinearVelocity exitVelocity,
      Angle launchAngle,
      Time timeOfFlight,
      double distanceMeters,
      double frontEdgeDistanceMeters,
      double rpmLookupDistanceMeters) {}

  /**
   * Map-based launch model: apply lookahead from chassis velocity, then interpolate speed and angle
   * by distance.
   */
  public static Optional<ShotSolution> calculateShotForHub(
      Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
    Translation2d launchTranslation =
        robotPose
            .getTranslation()
            .plus(
                new Translation2d(SHOOTER_OFFSET_X_METERS, 0.0).rotateBy(robotPose.getRotation()));

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
    double lookaheadFrontEdgeDistanceMeters =
        HUB_TRANSLATION.getDistance(lookaheadLaunchTranslation);
    double rpmLookupDistanceMeters = lookaheadFrontEdgeDistanceMeters;

    if (rpmLookupDistanceMeters < minDistanceMeters
        || rpmLookupDistanceMeters > maxDistanceMeters) {
      return Optional.empty();
    }

    double speedMetersPerSecond =
        MathUtil.clamp(
            exitVelocityMap.get(rpmLookupDistanceMeters),
            MIN_DYNAMIC_EXIT_VELOCITY.in(MetersPerSecond),
            MAX_DYNAMIC_EXIT_VELOCITY.in(MetersPerSecond));
    Angle launchAngle = fixedLaunchAngle;

    return Optional.of(
        new ShotSolution(
            MetersPerSecond.of(speedMetersPerSecond),
            launchAngle,
            Seconds.of(timeOfFlightMap.get(rpmLookupDistanceMeters)),
            lookaheadDistanceMeters,
            lookaheadFrontEdgeDistanceMeters,
            rpmLookupDistanceMeters));
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

  public static double rpmToVelocity(double flywheelRpm) {
    return flywheelVelocityToExitVelocity(
            RadiansPerSecond.of(Units.rotationsPerMinuteToRadiansPerSecond(flywheelRpm)))
        .in(MetersPerSecond);
  }
}
