package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.FieldConstants;

public final class ShooterMath {
  /** One row of the shot table: flywheel RPM, hood launch angle, and time of flight. */
  public record ShotParameters(double rpm, double angleDeg, double tofSeconds)
      implements Interpolatable<ShotParameters> {
    @Override
    public ShotParameters interpolate(ShotParameters end, double t) {
      return new ShotParameters(
          MathUtil.interpolate(rpm, end.rpm, t),
          MathUtil.interpolate(angleDeg, end.angleDeg, t),
          MathUtil.interpolate(tofSeconds, end.tofSeconds, t));
    }
  }

  /**
   * Shot table keyed by horizontal distance (m) from the shooter exit to the target. Values outside
   * the table are clamped to the nearest end, so the robot always produces a shot.
   *
   * <p>RPM column was tuned on the robot with the hood fixed at 59 deg; the angle column is 59 deg
   * throughout until per-distance hood angles are tuned.
   */
  private static final InterpolatingTreeMap<Double, ShotParameters> shotTable =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShotParameters::interpolate);

  private static final double minDistanceMeters;
  private static final double maxDistanceMeters;

  static {
    shotTable.put(Units.inchesToMeters(90), new ShotParameters(2650, 59.0, 0.86));
    shotTable.put(Units.inchesToMeters(100), new ShotParameters(2700, 59.0, 0.93));
    shotTable.put(Units.inchesToMeters(110), new ShotParameters(2800, 59.0, 1.00));
    shotTable.put(Units.inchesToMeters(120), new ShotParameters(2900, 59.0, 1.06));
    shotTable.put(Units.inchesToMeters(130), new ShotParameters(3000, 59.0, 1.13));
    shotTable.put(Units.inchesToMeters(140), new ShotParameters(3100, 59.0, 1.20));
    shotTable.put(Units.inchesToMeters(150), new ShotParameters(3250, 59.0, 1.25));
    shotTable.put(Units.inchesToMeters(160), new ShotParameters(3300, 59.0, 1.32));
    shotTable.put(Units.inchesToMeters(170), new ShotParameters(3400, 59.0, 1.39));
    shotTable.put(Units.inchesToMeters(180), new ShotParameters(3500, 59.0, 1.45));
    shotTable.put(Units.inchesToMeters(190), new ShotParameters(3650, 59.0, 1.45));
    shotTable.put(Units.inchesToMeters(200), new ShotParameters(3750, 59.0, 1.45));
    shotTable.put(Units.inchesToMeters(210), new ShotParameters(3800, 59.0, 1.45));

    minDistanceMeters = Units.inchesToMeters(90);
    maxDistanceMeters = Units.inchesToMeters(210);
  }

  private ShooterMath() {}

  public record ShotSolution(
      Translation2d target,
      LinearVelocity exitVelocity,
      AngularVelocity flywheelVelocity,
      Angle launchAngle,
      Time timeOfFlight,
      double distanceMeters,
      boolean inRange) {}

  /** Field position of the shooter exit for the given robot pose. */
  public static Translation2d getLaunchTranslation(Pose2d robotPose) {
    return robotPose
        .getTranslation()
        .plus(new Translation2d(SHOOTER_OFFSET_X_METERS, 0.0).rotateBy(robotPose.getRotation()));
  }

  /** The point the shooter should aim at from the given robot pose (hub or corner fallback). */
  public static Translation2d getTarget(Pose2d robotPose) {
    return FieldConstants.targetPosition(robotPose.getTranslation());
  }

  /**
   * Heading the robot must face so its (rear-facing) shooter points at the target, optionally from
   * a lookahead launch position.
   */
  public static Rotation2d getAimHeading(Translation2d launchTranslation, Translation2d target) {
    return target.minus(launchTranslation).getAngle().plus(Rotation2d.kPi);
  }

  /** Signed heading error (rad) between the robot's current heading and the aim heading. */
  public static double getAimErrorRad(Pose2d robotPose) {
    Rotation2d aim = getAimHeading(getLaunchTranslation(robotPose), getTarget(robotPose));
    return aim.minus(robotPose.getRotation()).getRadians();
  }

  public static boolean isAimed(Pose2d robotPose) {
    return Math.abs(getAimErrorRad(robotPose)) <= AIM_TOLERANCE_RAD;
  }

  /**
   * Map-based launch model: apply lookahead from chassis velocity (iterating on time of flight),
   * then interpolate RPM, hood angle, and TOF by distance.
   */
  public static ShotSolution calculateShot(Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
    Translation2d launchTranslation = getLaunchTranslation(robotPose);
    Translation2d target = getTarget(robotPose);

    Translation2d lookaheadLaunch = launchTranslation;
    double distance = target.getDistance(lookaheadLaunch);
    for (int i = 0; i < 8; i++) {
      double tof = shotTable.get(clampDistance(distance)).tofSeconds();
      lookaheadLaunch =
          launchTranslation.plus(
              new Translation2d(
                  fieldRelativeSpeeds.vxMetersPerSecond * tof,
                  fieldRelativeSpeeds.vyMetersPerSecond * tof));
      distance = target.getDistance(lookaheadLaunch);
    }

    boolean inRange = distance >= minDistanceMeters && distance <= maxDistanceMeters;
    ShotParameters params = shotTable.get(clampDistance(distance));

    AngularVelocity flywheel = RPM.of(params.rpm());
    LinearVelocity exitVelocity =
        MetersPerSecond.of(
            MathUtil.clamp(
                flywheelVelocityToExitVelocity(flywheel).in(MetersPerSecond),
                MIN_DYNAMIC_EXIT_VELOCITY.in(MetersPerSecond),
                MAX_DYNAMIC_EXIT_VELOCITY.in(MetersPerSecond)));

    return new ShotSolution(
        target,
        exitVelocity,
        flywheel,
        Degrees.of(params.angleDeg()),
        Seconds.of(params.tofSeconds()),
        distance,
        inRange);
  }

  private static double clampDistance(double distance) {
    return MathUtil.clamp(distance, minDistanceMeters, maxDistanceMeters);
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
