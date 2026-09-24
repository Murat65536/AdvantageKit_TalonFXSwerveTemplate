package frc.robot.fuelphysics;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Test-only port of team 6328's FUEL physics, used to validate shot tables against realistic
 * ballistics before they are trusted on a real robot.
 *
 * <p>Ported from {@code org.littletonrobotics.frc2026.util.FuelSim} in <a
 * href="https://github.com/Mechanical-Advantage/RobotCode2026Public">RobotCode2026Public</a>, which
 * is itself adapted from <a
 * href="https://github.com/hammerheads5000/FuelSim">hammerheads5000/FuelSim</a> (Copyright (c) 2026
 * LordOfFrogs, MIT license). Constants and the integration scheme are kept byte-for-byte faithful
 * so results stay comparable to 6328's.
 *
 * <h2>Why this exists</h2>
 *
 * <p>The robot's simulator (maple-sim) uses {@code GRAVITY = 11.0 m/s^2} and <b>no air drag</b> --
 * the inflated gravity is a single scalar standing in for the entire drag term. It also scores a
 * shot by testing whether the fuel's centre is inside a static box, with no rim, no tower, and no
 * requirement that the fuel is even falling.
 *
 * <p>Both simplifications flatter a shot table. This model instead uses real gravity (9.81) with
 * quadratic drag, and scores only when the fuel <b>descends through the hub entry plane inside the
 * opening</b> -- so a shot can realistically clip the tower or sail over into the net.
 *
 * <h2>What is modelled</h2>
 *
 * <ul>
 *   <li>Projectile flight: gravity + quadratic air drag, integrated at 6328's 4 ms sub-tick.
 *   <li>Hub scoring: downward crossing of the entry plane within the entry radius.
 *   <li>Hub tower: a solid box the fuel bounces off if it arrives too low.
 *   <li>Hub net: the backstop behind the goal that catches overshoots.
 * </ul>
 *
 * <h2>What is deliberately NOT modelled</h2>
 *
 * <p>Fuel-to-fuel collisions, robot collisions, intakes, trenches and floor bounces. Those matter
 * for loose fuel on the field, not for judging whether a launched shot goes in, and omitting them
 * keeps this validator deterministic.
 */
public final class FuelPhysics {

  // ---- Integration (FuelSim.PERIOD / FuelSim.subticks) ----

  /** Robot loop period, matching {@code Constants.loopPeriodSecs}. */
  public static final double PERIOD = 0.02;

  /** Physics sub-ticks per loop, matching {@code FuelSim.subticks}. */
  public static final int SUBTICKS = 5;

  /** Integration step: 4 ms. */
  public static final double DT = PERIOD / SUBTICKS;

  // ---- Fuel properties (FuelSim) ----

  /** Real gravity. Note maple-sim uses 11.0 to stand in for the drag this model computes. */
  public static final Translation3d GRAVITY = new Translation3d(0.0, 0.0, -9.81);

  /** Room-temperature dry air density, kg/m^3. */
  public static final double AIR_DENSITY = 1.2041;

  public static final double FUEL_RADIUS = 0.075;

  public static final double FUEL_MASS = 0.448 * 0.45392;

  public static final double FUEL_CROSS_AREA = Math.PI * FUEL_RADIUS * FUEL_RADIUS;

  /** Drag coefficient of a smooth sphere. */
  public static final double DRAG_COF = 0.47;

  public static final double DRAG_FORCE_FACTOR = 0.5 * AIR_DENSITY * DRAG_COF * FUEL_CROSS_AREA;

  /** Coefficient of restitution with field structure. */
  public static final double FIELD_COR = Math.sqrt(22 / 51.5);

  /** Coefficient of restitution with the net. */
  public static final double NET_COR = 0.2;

  // ---- Hub geometry (FuelSim.Hub) ----

  /** Height of the hub opening. Independently corroborated: maple-sim's box top is 1.8288 m. */
  public static final double ENTRY_HEIGHT = 1.83;

  /** Radius of the hub opening. */
  public static final double ENTRY_RADIUS = 0.56;

  /** Side length of the solid hub tower. */
  public static final double TOWER_SIDE = 1.2;

  /** Top of the solid tower, just below the entry plane. */
  public static final double TOWER_TOP = ENTRY_HEIGHT - 0.1;

  public static final double NET_HEIGHT_MAX = 3.057;
  public static final double NET_HEIGHT_MIN = 1.5;
  public static final double NET_OFFSET = TOWER_SIDE / 2 + 0.261;
  public static final double NET_WIDTH = 1.484;

  private FuelPhysics() {}

  /** How a simulated shot ended. */
  public enum Outcome {
    /** Descended through the hub opening. */
    SCORED,
    /** Struck the solid hub tower -- arrived too low. */
    HIT_TOWER,
    /** Caught by the net behind the hub -- overshot. */
    HIT_NET,
    /** Fell to the floor without reaching the hub. */
    FELL_SHORT,
    /** Still airborne when the simulation window expired. */
    TIMEOUT
  }

  /**
   * Result of a simulated shot.
   *
   * @param outcome how the shot ended
   * @param entryPlaneMiss horizontal distance from the hub axis at the moment the fuel descended
   *     through {@link #ENTRY_HEIGHT}; {@link Double#NaN} if it never crossed while falling. Values
   *     below {@link #ENTRY_RADIUS} score, so this is the shot's true error metric.
   * @param timeOfFlight seconds from launch to the outcome
   * @param apexHeight highest point reached, metres
   */
  public record Result(
      Outcome outcome, double entryPlaneMiss, double timeOfFlight, double apexHeight) {
    public boolean scored() {
      return outcome == Outcome.SCORED;
    }
  }

  /**
   * Flies one fuel and reports what happened, using 6328's integration scheme exactly: position is
   * advanced with the current velocity, then velocity is advanced by acceleration.
   *
   * @param launchPosition field-relative XY of the shooter exit
   * @param launchHeight height of the shooter exit, metres
   * @param headingRad field-relative direction of travel
   * @param exitSpeed speed leaving the shooter, m/s
   * @param launchAngleRad hood angle above horizontal
   * @param hubCenter field-relative XY of the hub axis
   */
  public static Result simulateShot(
      Translation2d launchPosition,
      double launchHeight,
      double headingRad,
      double exitSpeed,
      double launchAngleRad,
      Translation2d hubCenter) {

    double horizontalSpeed = exitSpeed * Math.cos(launchAngleRad);
    Translation3d pos =
        new Translation3d(launchPosition.getX(), launchPosition.getY(), launchHeight);
    Translation3d vel =
        new Translation3d(
            horizontalSpeed * Math.cos(headingRad),
            horizontalSpeed * Math.sin(headingRad),
            exitSpeed * Math.sin(launchAngleRad));

    double apex = launchHeight;
    double entryPlaneMiss = Double.NaN;
    Outcome collision = null;
    double collisionTime = 0.0;

    for (int step = 0; step < (int) (6.0 / DT); step++) {
      double t = step * DT;

      // --- FuelSim.Fuel.update: advance position, then velocity ---
      pos = pos.plus(vel.times(DT));
      if (pos.getZ() > FUEL_RADIUS) {
        Translation3d weight = GRAVITY.times(FUEL_MASS);
        Translation3d drag = Translation3d.kZero;
        double speed = vel.getNorm();
        if (speed > 1e-6) {
          drag = vel.times(-DRAG_FORCE_FACTOR * speed);
        }
        vel = vel.plus(weight.plus(drag).div(FUEL_MASS).times(DT));
      }
      apex = Math.max(apex, pos.getZ());

      // --- FuelSim.Hub.didFuelScore: descending crossing of the entry plane ---
      double radial = pos.toTranslation2d().getDistance(hubCenter);
      boolean wasAbove = pos.minus(vel.times(DT)).getZ() > ENTRY_HEIGHT;
      if (pos.getZ() <= ENTRY_HEIGHT && wasAbove) {
        entryPlaneMiss = radial;
        if (radial <= ENTRY_RADIUS) {
          return new Result(Outcome.SCORED, radial, t, apex);
        }
      }

      // --- Net backstop (FuelSim.Hub.fuelHitNet) ---
      if (collision == null && hitsNet(pos, hubCenter)) {
        collision = Outcome.HIT_NET;
        collisionTime = t;
      }

      // --- Solid tower (FuelSim.Hub.fuelCollideSide) ---
      if (collision == null && hitsTower(pos, hubCenter)) {
        collision = Outcome.HIT_TOWER;
        collisionTime = t;
      }

      if (pos.getZ() <= FUEL_RADIUS) {
        if (collision != null) {
          return new Result(collision, entryPlaneMiss, collisionTime, apex);
        }
        return new Result(Outcome.FELL_SHORT, entryPlaneMiss, t, apex);
      }
    }

    if (collision != null) {
      return new Result(collision, entryPlaneMiss, collisionTime, apex);
    }
    return new Result(Outcome.TIMEOUT, entryPlaneMiss, 6.0, apex);
  }

  /** Mirrors {@code FuelSim.fuelCollideRectangle} applied to the hub tower. */
  private static boolean hitsTower(Translation3d pos, Translation2d hubCenter) {
    if (pos.getZ() > TOWER_TOP + FUEL_RADIUS) {
      return false;
    }
    double half = TOWER_SIDE / 2.0;
    double dx = Math.abs(pos.getX() - hubCenter.getX());
    double dy = Math.abs(pos.getY() - hubCenter.getY());
    return dx <= half + FUEL_RADIUS && dy <= half + FUEL_RADIUS;
  }

  /** Mirrors {@code FuelSim.Hub.fuelHitNet} for a shot travelling in +x toward the hub. */
  private static boolean hitsNet(Translation3d pos, Translation2d hubCenter) {
    if (pos.getZ() > NET_HEIGHT_MAX || pos.getZ() < NET_HEIGHT_MIN) {
      return false;
    }
    if (Math.abs(pos.getY() - hubCenter.getY()) > NET_WIDTH / 2.0) {
      return false;
    }
    return pos.getX() + FUEL_RADIUS >= hubCenter.getX() + NET_OFFSET;
  }
}
