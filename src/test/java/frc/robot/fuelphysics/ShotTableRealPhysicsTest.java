package frc.robot.fuelphysics;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.shooter.ShooterMath;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Judges the shipped shot table against realistic ballistics (real gravity + quadratic drag, and a
 * hub you can actually miss) instead of the simulator's forgiving physics.
 *
 * <p>This is the guard that stops a table tuned to maple-sim's {@code GRAVITY = 11.0} and static
 * box check from being trusted on a real robot.
 */
public class ShotTableRealPhysicsTest {

  private static final Translation2d HUB = new Translation2d(4.5974, 4.034536);
  private static final double LAUNCH_HEIGHT = Units.inchesToMeters(21.2315);
  private static final double SLIP = Units.inchesToMeters(2.0) * 0.58;

  /**
   * Required tolerance to flywheel error, each way. The flywheel's own readiness gate is {@code
   * SHOOTER_AT_SPEED_TOLERANCE = 150 RPM}, so a table that scores over a narrower band than that
   * can be "ready" and still miss.
   */
  private static final double REQUIRED_RPM_MARGIN = 40.0;

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

  private static FuelPhysics.Result shootFrom(double distanceMeters, double rpm, double angleDeg) {
    Translation2d launch = new Translation2d(HUB.getX() - distanceMeters, HUB.getY());
    return FuelPhysics.simulateShot(
        launch, LAUNCH_HEIGHT, 0.0, rpmToExitVelocity(rpm), Math.toRadians(angleDeg), HUB);
  }

  @Test
  void everyTableDistanceScoresUnderRealPhysics() {
    StringBuilder report = new StringBuilder("\n");
    boolean allScored = true;

    for (int distInches = 60; distInches <= 260; distInches += 10) {
      double distance = Units.inchesToMeters(distInches);
      var params = ShooterMath.getShotParameters(distance);
      FuelPhysics.Result r = shootFrom(distance, params.rpm(), params.angleDeg());

      allScored &= r.scored();
      report.append(
          String.format(
              "  %3din  rpm=%4.0f ang=%4.1f  ->  %-10s miss=%.3fm (opening r=%.2fm) tof=%.3fs apex=%.2fm%n",
              distInches,
              params.rpm(),
              params.angleDeg(),
              r.outcome(),
              r.entryPlaneMiss(),
              FuelPhysics.ENTRY_RADIUS,
              r.timeOfFlight(),
              r.apexHeight()));
    }

    System.out.print(report);
    assertTrue(allScored, "table misses under real physics:" + report);
  }

  @Test
  void everyTableDistanceHasUsableFlywheelMargin() {
    StringBuilder report = new StringBuilder("\n");
    boolean allOk = true;

    for (int distInches = 60; distInches <= 260; distInches += 10) {
      double distance = Units.inchesToMeters(distInches);
      var params = ShooterMath.getShotParameters(distance);

      double up = 0;
      while (up < 400 && shootFrom(distance, params.rpm() + up + 5, params.angleDeg()).scored()) {
        up += 5;
      }
      double down = 0;
      while (down < 400
          && shootFrom(distance, params.rpm() - down - 5, params.angleDeg()).scored()) {
        down += 5;
      }

      boolean ok = up >= REQUIRED_RPM_MARGIN && down >= REQUIRED_RPM_MARGIN;
      allOk &= ok;
      report.append(
          String.format(
              "  %3din  rpm=%4.0f  margin +%3.0f / -%3.0f RPM  %s%n",
              distInches, params.rpm(), up, down, ok ? "ok" : "TOO TIGHT"));
    }

    System.out.print(report);
    assertTrue(
        allOk,
        "some distances score only within a razor-thin RPM band, so the flywheel's own "
            + "150 RPM readiness gate would pass shots that miss:"
            + report);
  }

  @Test
  void aimErrorBudgetIsDocumented() {
    // How far off heading can the robot be at mid range before the shot leaves the opening?
    double distance = Units.inchesToMeters(150);
    var params = ShooterMath.getShotParameters(distance);

    double worstGood = 0.0;
    for (double errDeg = 0.0; errDeg <= 15.0; errDeg += 0.25) {
      Translation2d launch = new Translation2d(HUB.getX() - distance, HUB.getY());
      FuelPhysics.Result r =
          FuelPhysics.simulateShot(
              launch,
              LAUNCH_HEIGHT,
              Math.toRadians(errDeg),
              rpmToExitVelocity(params.rpm()),
              Math.toRadians(params.angleDeg()),
              HUB);
      if (r.scored()) {
        worstGood = errDeg;
      } else {
        break;
      }
    }

    System.out.printf(
        "[aim] at 150in the shot tolerates up to %.2f deg of heading error "
            + "(AIM_TOLERANCE_RAD is 5.00 deg)%n",
        worstGood);
    assertTrue(worstGood > 0.0, "no heading error at all is tolerated -- suspicious");
  }
}
