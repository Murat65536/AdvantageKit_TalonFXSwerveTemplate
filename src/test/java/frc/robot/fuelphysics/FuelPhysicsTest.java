package frc.robot.fuelphysics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Sanity checks on the {@link FuelPhysics} port itself, so the validator is trusted before any shot
 * table is judged by it. These test the MODEL, not the robot.
 */
public class FuelPhysicsTest {

  private static final Translation2d HUB = new Translation2d(4.5974, 4.034536);

  @BeforeEach
  void setUp() {
    assert HAL.initialize(500, 0);
  }

  @AfterEach
  void tearDown() {
    HAL.shutdown();
  }

  @Test
  void constantsMatchSourceExactly() {
    // Guards against a transcription slip in the port.
    assertEquals(0.004, FuelPhysics.DT, 1e-12, "sub-tick must be 4ms");
    assertEquals(-9.81, FuelPhysics.GRAVITY.getZ(), 1e-12, "must use REAL gravity, not 11.0");
    assertEquals(0.20336, FuelPhysics.FUEL_MASS, 1e-5);
    assertEquals(0.005, FuelPhysics.DRAG_FORCE_FACTOR, 1e-4);
    assertEquals(1.83, FuelPhysics.ENTRY_HEIGHT, 1e-12);
    assertEquals(0.56, FuelPhysics.ENTRY_RADIUS, 1e-12);
  }

  @Test
  void terminalVelocityIsPhysicallySane() {
    double terminal = Math.sqrt(FuelPhysics.FUEL_MASS * 9.81 / FuelPhysics.DRAG_FORCE_FACTOR);
    // A 150mm foam-ish ball: tens of m/s, not hundreds, not single digits.
    assertTrue(terminal > 15.0 && terminal < 30.0, "terminal velocity = " + terminal);
  }

  @Test
  void dragMakesShotsFallShorterThanVacuum() {
    // Same launch, drag on (this model) vs the drag-free analytic range.
    double v = 12.0;
    double angle = Math.toRadians(45.0);
    double h0 = 0.53927;

    FuelPhysics.Result r =
        FuelPhysics.simulateShot(
            new Translation2d(0.0, HUB.getY()),
            h0,
            0.0,
            v,
            angle,
            new Translation2d(100.0, HUB.getY()));

    // Vacuum range with g=9.81 from height h0.
    double vx = v * Math.cos(angle);
    double vz = v * Math.sin(angle);
    double tVac = (vz + Math.sqrt(vz * vz + 2 * 9.81 * h0)) / 9.81;
    double vacuumRange = vx * tVac;

    // With drag the ball must land measurably shorter, and still travel a sane distance.
    assertTrue(r.outcome() == FuelPhysics.Outcome.FELL_SHORT, "expected a floor landing");
    double draggedRange = vx * r.timeOfFlight();
    assertTrue(
        draggedRange < vacuumRange,
        String.format("drag range %.2f should be < vacuum %.2f", draggedRange, vacuumRange));
  }

  @Test
  void aShotAimedAtTheOpeningScores() {
    // Fired from 3.81m at a speed/angle solved for the opening: must go in.
    double distance = 3.81;
    Translation2d launch = new Translation2d(HUB.getX() - distance, HUB.getY());

    boolean anyScored = false;
    for (double rpm = 1800; rpm <= 3200; rpm += 5) {
      double v = rpm * 2.0 * Math.PI / 60.0 * (0.0508 * 0.58);
      FuelPhysics.Result r =
          FuelPhysics.simulateShot(launch, 0.53927, 0.0, v, Math.toRadians(50.0), HUB);
      if (r.scored()) {
        anyScored = true;
        break;
      }
    }
    assertTrue(anyScored, "no speed in a wide sweep could score -- the model is broken");
  }

  @Test
  void aWildlyOverspeedShotDoesNotScore() {
    Translation2d launch = new Translation2d(HUB.getX() - 3.81, HUB.getY());
    FuelPhysics.Result r =
        FuelPhysics.simulateShot(launch, 0.53927, 0.0, 20.0, Math.toRadians(50.0), HUB);
    assertTrue(!r.scored(), "a 20 m/s shot should sail past the goal, got " + r);
  }

  @Test
  void aWildlyUnderspeedShotFallsShort() {
    Translation2d launch = new Translation2d(HUB.getX() - 3.81, HUB.getY());
    FuelPhysics.Result r =
        FuelPhysics.simulateShot(launch, 0.53927, 0.0, 4.0, Math.toRadians(50.0), HUB);
    assertTrue(!r.scored(), "a 4 m/s shot cannot reach the goal, got " + r);
  }
}
