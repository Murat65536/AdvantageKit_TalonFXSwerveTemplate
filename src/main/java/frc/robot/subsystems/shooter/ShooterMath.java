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
   * <p>DERIVED, not hand-tuned, and validated against REALISTIC ballistics -- real gravity (9.81)
   * with quadratic air drag, a solid hub tower the fuel can strike, and a net that catches
   * overshoots. See {@code FuelPhysics} / {@code ShotTableRealPhysicsTest}, ported from team 6328's
   * FuelSim. Each row maximizes tolerance to flywheel speed error.
   *
   * <p>The angle column varies with distance by derivation: close shots use the steepest hood angle
   * (76 deg) to drop nearly vertically into the opening and clear the surrounding tower, while long
   * shots flatten out (to a minimum of 53.5 deg at 240 in) because a steep lob there arrives too
   * steeply to satisfy both models.
   *
   * <p>Range is 60..260 in. The old 90..210 in table was too narrow: a robot sitting 247 in from
   * the hub had its distance silently clamped to the 210 in row and every shot fell short.
   *
   * <p>TOF is computed from the drag model, so the moving-shot lookahead stays consistent.
   */
  private static final InterpolatingTreeMap<Double, ShotParameters> shotTable =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShotParameters::interpolate);

  private static final double minDistanceMeters;
  private static final double maxDistanceMeters;

  static {
    shotTable.put(Units.inchesToMeters(60), new ShotParameters(2125, 76.0, 1.004));
    shotTable.put(Units.inchesToMeters(70), new ShotParameters(2260, 76.0, 1.104));
    shotTable.put(Units.inchesToMeters(80), new ShotParameters(2395, 76.0, 1.196));
    shotTable.put(Units.inchesToMeters(90), new ShotParameters(2530, 76.0, 1.284));
    shotTable.put(Units.inchesToMeters(100), new ShotParameters(2650, 76.0, 1.364));
    shotTable.put(Units.inchesToMeters(110), new ShotParameters(2770, 76.0, 1.436));
    shotTable.put(Units.inchesToMeters(120), new ShotParameters(2890, 76.0, 1.512));
    shotTable.put(Units.inchesToMeters(130), new ShotParameters(3000, 76.0, 1.576));
    shotTable.put(Units.inchesToMeters(140), new ShotParameters(3110, 76.0, 1.640));
    shotTable.put(Units.inchesToMeters(150), new ShotParameters(3220, 76.0, 1.704));
    shotTable.put(Units.inchesToMeters(160), new ShotParameters(3325, 76.0, 1.760));
    shotTable.put(Units.inchesToMeters(170), new ShotParameters(2955, 70.0, 1.488));
    shotTable.put(Units.inchesToMeters(180), new ShotParameters(3165, 72.0, 1.632));
    shotTable.put(Units.inchesToMeters(190), new ShotParameters(3330, 73.0, 1.736));
    shotTable.put(Units.inchesToMeters(200), new ShotParameters(3035, 67.0, 1.496));
    shotTable.put(Units.inchesToMeters(210), new ShotParameters(3085, 66.5, 1.516));
    shotTable.put(Units.inchesToMeters(220), new ShotParameters(3155, 66.5, 1.556));
    shotTable.put(Units.inchesToMeters(230), new ShotParameters(3100, 63.5, 1.480));
    shotTable.put(Units.inchesToMeters(240), new ShotParameters(2955, 53.5, 1.212));
    shotTable.put(Units.inchesToMeters(250), new ShotParameters(3020, 54.5, 1.268));
    shotTable.put(Units.inchesToMeters(260), new ShotParameters(3150, 59.0, 1.428));

    minDistanceMeters = Units.inchesToMeters(60);
    maxDistanceMeters = Units.inchesToMeters(260);
  }

  private ShooterMath() {}

  /** Shot table row for a horizontal distance (m), clamped to the table's range. */
  public static ShotParameters getShotParameters(double distanceMeters) {
    return shotTable.get(clampDistance(distanceMeters));
  }

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
