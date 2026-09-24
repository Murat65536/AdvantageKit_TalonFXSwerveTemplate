package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterMath;
import java.util.Optional;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Verifies the shot table actually puts fuel through maple-sim's scoring box, by replaying the same
 * physics maple-sim integrates (constant horizontal velocity, g = 11.0, no drag).
 */
public class ShotTableScoringTest {

  /**
   * maple-sim's {@code GamePieceProjectile.GRAVITY}. It is 11.0, not 9.81, on purpose: the
   * simulator models no air drag and inflates gravity as a single scalar standing in for the
   * missing drag term. Do not "fix" this to 9.81 -- that would make every shot in this test fly
   * long. The realistic model lives in {@code FuelPhysics} and uses real gravity plus drag.
   */
  private static final double MAPLE_GRAVITY = 11.0;

  private static final double LAUNCH_HEIGHT_METERS = ShooterConstants.BALL_EXIT_TRANSLATION.getZ();
  private static final double SLIP = ShooterConstants.EXIT_VELOCITY_PER_FLYWHEEL_RAD_PER_SEC;
  private static final double HUB_HALF_WIDTH_METERS = FieldConstants.HUB_LENGTH / 2.0;

  @BeforeEach
  void setUp() {
    assert HAL.initialize(500, 0);
  }

  @AfterEach
  void tearDown() {
    HAL.shutdown();
  }

  private static double rpmToExitVelocity(double rpm) {
    return rpm * 2.0 * Math.PI / 60.0 * SLIP;
  }

  /**
   * Replays a shot the way maple-sim actually scores it: stepping the projectile and checking
   * whether the fuel is ever inside the scoring box, rather than sampling a single instant.
   *
   * @return time of entry, or empty if the shot never enters the box
   */
  private static Optional<Double> mapleEntryTime(double distance, double rpm, double angleDeg) {
    double v = rpmToExitVelocity(rpm);
    double angle = Math.toRadians(angleDeg);
    double vx = v * Math.cos(angle);
    double vz0 = v * Math.sin(angle);

    for (double t = 0.0; t < 6.0; t += 0.004) {
      double x = vx * t;
      double z = LAUNCH_HEIGHT_METERS + vz0 * t - 0.5 * MAPLE_GRAVITY * t * t;
      if (z < 0.0) {
        return Optional.empty();
      }
      if (Math.abs(distance - x) <= HUB_HALF_WIDTH_METERS
          && z >= FieldConstants.HUB_SCORING_Z_MIN
          && z <= FieldConstants.HUB_SCORING_Z_MAX) {
        return Optional.of(t);
      }
    }
    return Optional.empty();
  }

  @Test
  void everyTableDistanceLandsInScoringBox() {
    StringBuilder report = new StringBuilder("\n");
    boolean allScored = true;

    for (int distInches = 60; distInches <= 260; distInches += 10) {
      double distance = Units.inchesToMeters(distInches);
      var params = ShooterMath.getShotParameters(distance);
      Optional<Double> entry = mapleEntryTime(distance, params.rpm(), params.angleDeg());

      allScored &= entry.isPresent();
      report.append(
          String.format(
              "  %3din  rpm=%4.0f ang=%4.1f  ->  %s%n",
              distInches,
              params.rpm(),
              params.angleDeg(),
              entry.map(t -> String.format("SCORE at t=%.3fs", t)).orElse("MISS")));
    }

    System.out.print(report);
    assertTrue(allScored, "some table distances miss maple-sim's scoring box:" + report);
  }

  @Test
  void tableTofIsConsistentWithDragModel() {
    // TOF now comes from the drag model, so it must exceed the drag-free flight time
    // (drag slows the fuel) while staying in the same ballpark.
    for (int distInches = 60; distInches <= 260; distInches += 10) {
      double distance = Units.inchesToMeters(distInches);
      var params = ShooterMath.getShotParameters(distance);
      double v = rpmToExitVelocity(params.rpm());
      double angleRad = Math.toRadians(params.angleDeg());
      double dragFreeTof = distance / (v * Math.cos(angleRad));

      assertTrue(
          params.tofSeconds() > 0.0 && params.tofSeconds() < 4.0,
          String.format("implausible TOF at %din: %.3f", distInches, params.tofSeconds()));
      assertTrue(
          params.tofSeconds() >= dragFreeTof - 0.05,
          String.format(
              "TOF at %din (%.3f) is shorter than the drag-free flight time (%.3f), "
                  + "which drag makes impossible",
              distInches, params.tofSeconds(), dragFreeTof));
    }
  }

  @Test
  void hoodAnglesAreWithinMechanicalRange() {
    for (int distInches = 60; distInches <= 260; distInches += 10) {
      var params = ShooterMath.getShotParameters(Units.inchesToMeters(distInches));
      assertTrue(
          params.angleDeg() >= 50.0 && params.angleDeg() <= 76.0,
          String.format(
              "hood angle %.1f at %din is outside the mechanical range 50..76",
              params.angleDeg(), distInches));
    }
  }

  @Test
  void aimingAtTowerHeightWouldOvershoot() {
    // Guards the bug this fixed: HUB_HEIGHT_METERS (104in) is the tower, not the scoring band.
    assertTrue(
        FieldConstants.HUB_HEIGHT_METERS > FieldConstants.HUB_SCORING_Z_MAX + 0.5,
        "tower height should be well above the scoring band");
  }
}
