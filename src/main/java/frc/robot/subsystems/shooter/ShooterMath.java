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
   * Shot table keyed by horizontal distance (m) from the shooter exit to the hub centre. Values
   * outside the table are clamped to the nearest end, so the robot always produces a shot.
   *
   * <p>MEASURED ON THE ROBOT: the RPM and hood angle of each row are the 2026RobotProject's shot
   * map ({@code ShooterConstants.SHOOTER_RPM_MAP}), tuned on this same robot. That project measures
   * distance the same way (barrel exit to hub centre) and drives the hood with the same angle
   * convention, so the rows carry over unchanged. Beyond 188 in it held 60 deg / 3500 RPM, which
   * the clamp reproduces.
   *
   * <p>The previous table was derived in simulation and asked for 76 deg (the steepest) from 60 to
   * 160 in, 3 to 14 deg steeper than these. On the robot those shots went high and missed. The sim
   * assumed an exit-velocity slip factor of 0.58; these real shots imply about 0.43 to 0.47.
   *
   * <p>TOF is NOT measured (the old project had none). It is the drag-free flight time that takes a
   * ball launched at the row's angle from the barrel height (21.2 in) through the top of the hub
   * opening (72 in) at the row's distance. Every row arrives descending, as a scoring shot must.
   * Only the moving-shot lookahead and the hub-schedule feed gate use it.
   */
  private static final InterpolatingTreeMap<Double, ShotParameters> shotTable =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShotParameters::interpolate);

  private static final double minDistanceMeters;
  private static final double maxDistanceMeters;

  static {
    shotTable.put(Units.inchesToMeters(52), new ShotParameters(2500, 73.0, 0.786));
    shotTable.put(Units.inchesToMeters(120), new ShotParameters(2800, 65.0, 1.034));
    shotTable.put(Units.inchesToMeters(188), new ShotParameters(3500, 60.0, 1.193));

    minDistanceMeters = Units.inchesToMeters(52);
    maxDistanceMeters = Units.inchesToMeters(188);
  }

  private ShooterMath() {}

  /** Nearest distance (m) the table covers; closer shots use this row. */
  public static double getMinDistanceMeters() {
    return minDistanceMeters;
  }

  /** Farthest distance (m) the table covers; farther shots use this row. */
  public static double getMaxDistanceMeters() {
    return maxDistanceMeters;
  }

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

  /** True when a shot from {@code distanceMeters} is covered by the table (not clamped). */
  public static boolean isInTableRange(double distanceMeters) {
    return distanceMeters >= minDistanceMeters && distanceMeters <= maxDistanceMeters;
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
