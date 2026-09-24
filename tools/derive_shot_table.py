#!/usr/bin/env python3
"""Derive the Wescavator shot table from physics, for a measured slip factor.

Every row is solved so the shot scores under BOTH models the robot is judged by:

  * maple-sim  -- the simulator the robot actually runs against.
                  g = 11.0 m/s^2, NO air drag (the inflated gravity is a single
                  scalar standing in for the missing drag term). A shot counts
                  when the fuel's centre is anywhere inside a 47x47x10 inch box.

  * FuelSim    -- team 6328's realistic model (they do not use maple-sim).
                  Real gravity 9.81 with quadratic air drag, a solid hub tower
                  the fuel can strike, and a net that catches overshoots. A shot
                  counts only when the fuel DESCENDS through the hub opening.

Requiring both keeps the table honest: maple-sim alone is too forgiving (no
tower, no drag), and FuelSim alone would let the table drift away from the
simulator the code is developed in.

For each distance the script searches the hood's mechanical range and picks the
angle whose scoring RPM band is widest, then takes the centre of that band. That
maximizes tolerance to flywheel speed error rather than aiming at a knife edge.

WHY THIS SCRIPT EXISTS
----------------------
Everything here is derived from sourced constants except ONE measured input:
the flywheel-to-exit-velocity slip factor. Exit speed is

    v_exit = omega_flywheel * FLYWHEEL_EFFECTIVE_RADIUS * SLIP_FACTOR

Every RPM in the table scales directly with that factor, so when you measure it
on the real robot, re-run this script and the whole table follows.

USAGE
-----
  # numpy is required; on NixOS:
  nix-shell -p 'python3.withPackages(ps: [ps.numpy])' \
    --run "python3 tools/derive_shot_table.py --slip-factor 0.58"

  # write the Java block straight into ShooterMath.java
  ... --slip-factor 0.58 --write

  # verify the committed table still scores (exit 1 if not)
  ... --slip-factor 0.58 --check
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from dataclasses import dataclass, asdict
from pathlib import Path

try:
    import numpy as np
except ImportError:
    sys.exit(
        "numpy is required.\n"
        "  NixOS:  nix-shell -p 'python3.withPackages(ps: [ps.numpy])' "
        "--run \"python3 tools/derive_shot_table.py --slip-factor 0.58\"\n"
        "  other:  pip install numpy"
    )

IN = 0.0254

# ---------------------------------------------------------------------------
# Sourced constants. Each cites where it comes from -- none are guesses.
# ---------------------------------------------------------------------------

# --- Robot geometry (ShooterConstants.BALL_EXIT_TRANSLATION) ---
LAUNCH_HEIGHT = 21.2315 * IN
FLYWHEEL_RADIUS = 2.0 * IN
MAX_FLYWHEEL_RPM = 6784.0  # Neo Vortex free speed

# --- Hood range (HoodConstants MIN/MAX_HOOD_ANGLE) ---
MIN_HOOD_DEG = 50.0
MAX_HOOD_DEG = 76.0

# --- maple-sim scoring (RebuiltHub -> Goal(47in, 47in, 10in) @ z=1.5748) ---
# Goal.checkValidity uses positionChecker = box(...). RebuiltHub also defines a
# spherical checkCollision, but nothing calls it -- it is dead code.
MAPLE_GRAVITY = 11.0
MAPLE_BOX_HALF = 47.0 * IN / 2.0
MAPLE_Z_MIN = 1.5748
MAPLE_Z_MAX = MAPLE_Z_MIN + 10.0 * IN

# --- 6328 FuelSim constants (util/FuelSim.java) ---
REAL_GRAVITY = 9.81
AIR_DENSITY = 1.2041
FUEL_RADIUS = 0.075
FUEL_MASS = 0.448 * 0.45392
DRAG_COF = 0.47
DRAG_FORCE_FACTOR = 0.5 * AIR_DENSITY * DRAG_COF * math.pi * FUEL_RADIUS**2

ENTRY_HEIGHT = 1.83          # FuelSim.Hub.ENTRY_HEIGHT
ENTRY_RADIUS = 0.56          # FuelSim.Hub.ENTRY_RADIUS
TOWER_HALF = 1.2 / 2.0       # FuelSim.Hub.SIDE / 2
TOWER_TOP = ENTRY_HEIGHT - 0.1
NET_Z_MIN = 1.5              # FuelSim.Hub.NET_HEIGHT_MIN
NET_Z_MAX = 3.057            # FuelSim.Hub.NET_HEIGHT_MAX
NET_OFFSET = TOWER_HALF + 0.261

DT = 0.004                   # FuelSim: PERIOD 0.02 / subticks 5
MAX_FLIGHT_S = 6.0

DEFAULT_DISTANCES_IN = list(range(60, 261, 10))


@dataclass
class Row:
    distance_in: int
    angle_deg: float
    rpm: int
    margin_rpm: int
    tof_s: float


def rpm_to_exit_velocity(rpm, slip_factor):
    """Flywheel RPM -> fuel exit speed (m/s). The slip factor is the measured input."""
    return rpm * 2.0 * math.pi / 60.0 * FLYWHEEL_RADIUS * slip_factor


def exit_velocity_to_rpm(v, slip_factor):
    return v / (FLYWHEEL_RADIUS * slip_factor) * 60.0 / (2.0 * math.pi)


def _simulate(distance, angles_deg, speeds, model):
    """Vectorised flight of every (angle, speed) pair. Returns a boolean scored mask.

    Shots travel along +x toward a hub whose axis sits at ``distance``. Both
    models integrate with the same step so their verdicts are comparable.
    """
    ang = np.radians(angles_deg)
    vx = speeds * np.cos(ang)
    vz = speeds * np.sin(ang)
    x = np.zeros_like(vx)
    z = np.full_like(vx, LAUNCH_HEIGHT)

    scored = np.zeros(vx.shape, dtype=bool)
    done = np.zeros(vx.shape, dtype=bool)

    steps = int(MAX_FLIGHT_S / DT)
    for _ in range(steps):
        prev_z = z
        # Position advances on the current velocity, then velocity updates --
        # matching FuelSim.Fuel.update exactly.
        x = x + vx * DT
        z = z + vz * DT

        if model == "real":
            speed = np.hypot(vx, vz)
            vx = vx - DRAG_FORCE_FACTOR * speed * vx / FUEL_MASS * DT
            vz = vz + (-REAL_GRAVITY - DRAG_FORCE_FACTOR * speed * vz / FUEL_MASS) * DT
        else:
            vz = vz - MAPLE_GRAVITY * DT

        radial = np.abs(distance - x)
        live = ~done

        if model == "maple":
            hit = live & (radial <= MAPLE_BOX_HALF) & (z >= MAPLE_Z_MIN) & (z <= MAPLE_Z_MAX)
            scored |= hit
            done |= hit
            done |= live & (z < 0.0)
        else:
            # Descending crossing of the opening plane decides the shot.
            crossing = live & (z <= ENTRY_HEIGHT) & (prev_z > ENTRY_HEIGHT)
            scored |= crossing & (radial <= ENTRY_RADIUS)
            done |= crossing
            live = ~done
            done |= live & (z <= TOWER_TOP + FUEL_RADIUS) & (radial <= TOWER_HALF + FUEL_RADIUS)
            live = ~done
            done |= live & (z >= NET_Z_MIN) & (z <= NET_Z_MAX) & (x + FUEL_RADIUS >= distance + NET_OFFSET)
            live = ~done
            done |= live & (z <= FUEL_RADIUS)

        if done.all():
            break

    return scored


def time_of_flight(distance, angle_deg, rpm, slip_factor):
    """Flight time to the opening under the drag model, for moving-shot lookahead."""
    ang = math.radians(angle_deg)
    v = rpm_to_exit_velocity(rpm, slip_factor)
    x, z = 0.0, LAUNCH_HEIGHT
    vx, vz = v * math.cos(ang), v * math.sin(ang)
    t = 0.0
    while t < MAX_FLIGHT_S:
        prev_z = z
        x += vx * DT
        z += vz * DT
        t += DT
        speed = math.hypot(vx, vz)
        vx -= DRAG_FORCE_FACTOR * speed * vx / FUEL_MASS * DT
        vz += (-REAL_GRAVITY - DRAG_FORCE_FACTOR * speed * vz / FUEL_MASS) * DT
        if z <= ENTRY_HEIGHT and prev_z > ENTRY_HEIGHT:
            break
    return t


def widest_run(rpms, ok_mask):
    """Widest contiguous block of scoring RPMs -> (centre, half-width)."""
    best_len = best_start = cur_start = 0
    cur_len = 0
    for i, ok in enumerate(ok_mask):
        if ok:
            if cur_len == 0:
                cur_start = i
            cur_len += 1
            if cur_len > best_len:
                best_len, best_start = cur_len, cur_start
        else:
            cur_len = 0
    if best_len == 0:
        return None
    lo = rpms[best_start]
    hi = rpms[best_start + best_len - 1]
    return int((lo + hi) / 2), int((hi - lo) / 2)


def solve_distance(distance_in, slip_factor, angle_step, rpm_step, verbose):
    """Best (angle, rpm) at one distance: the pair with the widest dual-model band."""
    distance = distance_in * IN
    angles = np.arange(MIN_HOOD_DEG, MAX_HOOD_DEG + 1e-9, angle_step)
    rpms = np.arange(1000.0, MAX_FLYWHEEL_RPM + 1e-9, rpm_step)

    grid_ang, grid_rpm = np.meshgrid(angles, rpms, indexing="ij")
    flat_ang = grid_ang.ravel()
    flat_speed = rpm_to_exit_velocity(grid_rpm.ravel(), slip_factor)

    ok = _simulate(distance, flat_ang, flat_speed, "real") & _simulate(
        distance, flat_ang, flat_speed, "maple"
    )
    ok = ok.reshape(grid_ang.shape)

    best = None
    for i, angle in enumerate(angles):
        run = widest_run(rpms, ok[i])
        if run is None:
            continue
        rpm, margin = run
        # Many angles tie on band width. Break ties toward the STEEPER angle: it
        # clears the hub tower by more at close range and tolerates more heading
        # error, and it makes this script deterministic rather than dependent on
        # search order. `angles` ascends, so >= keeps the steepest tying angle.
        if best is None or margin >= best.margin_rpm:
            best = Row(distance_in, round(float(angle), 1), rpm, margin, 0.0)

    if best is not None:
        best.tof_s = round(time_of_flight(distance, best.angle_deg, best.rpm, slip_factor), 3)
    if verbose:
        if best is None:
            print(f"  {distance_in:>4}in   NO SOLUTION", file=sys.stderr)
        else:
            print(
                f"  {distance_in:>4}in   {best.angle_deg:>5.1f} deg  {best.rpm:>5} rpm  "
                f"+/-{best.margin_rpm:>3} rpm  tof={best.tof_s:.3f}s",
                file=sys.stderr,
            )
    return best


def render_java(rows):
    return "\n".join(
        f"    shotTable.put(Units.inchesToMeters({r.distance_in}), "
        f"new ShotParameters({r.rpm}, {r.angle_deg:.1f}, {r.tof_s:.3f}));"
        for r in rows
    )


SHOOTER_MATH = Path(__file__).resolve().parent.parent / (
    "src/main/java/frc/robot/subsystems/shooter/ShooterMath.java"
)


def write_into_shooter_math(rows):
    """Replace the shotTable.put block and the min/max range in ShooterMath.java."""
    if not SHOOTER_MATH.exists():
        sys.exit(f"cannot find {SHOOTER_MATH}")
    text = SHOOTER_MATH.read_text()

    block = re.search(r"( *shotTable\.put\(.*?\);\n)+", text, re.DOTALL)
    if not block:
        sys.exit("could not locate the shotTable.put block in ShooterMath.java")
    text = text[: block.start()] + render_java(rows) + "\n" + text[block.end() :]

    text = re.sub(
        r"minDistanceMeters = Units\.inchesToMeters\(\d+\);",
        f"minDistanceMeters = Units.inchesToMeters({rows[0].distance_in});",
        text,
    )
    text = re.sub(
        r"maxDistanceMeters = Units\.inchesToMeters\(\d+\);",
        f"maxDistanceMeters = Units.inchesToMeters({rows[-1].distance_in});",
        text,
    )
    SHOOTER_MATH.write_text(text)
    print(f"wrote {len(rows)} rows into {SHOOTER_MATH}", file=sys.stderr)
    print("run ./gradlew spotlessApply test to verify", file=sys.stderr)


def parse_committed_table():
    """Read the table currently in ShooterMath.java."""
    text = SHOOTER_MATH.read_text()
    rows = []
    for m in re.finditer(
        r"shotTable\.put\(Units\.inchesToMeters\((\d+)\),\s*"
        r"new ShotParameters\(([\d.]+),\s*([\d.]+),\s*([\d.]+)\)\);",
        text,
    ):
        rows.append(
            Row(int(m.group(1)), float(m.group(3)), int(float(m.group(2))), 0, float(m.group(4)))
        )
    return rows


def check_committed(slip_factor):
    """Verify every committed row still scores under both models."""
    rows = parse_committed_table()
    if not rows:
        sys.exit("no shot table found in ShooterMath.java")

    print(f"checking {len(rows)} committed rows at slip factor {slip_factor}\n")
    failures = []
    for r in rows:
        distance = r.distance_in * IN
        ang = np.array([r.angle_deg])
        speed = np.array([rpm_to_exit_velocity(r.rpm, slip_factor)])
        real = bool(_simulate(distance, ang, speed, "real")[0])
        maple = bool(_simulate(distance, ang, speed, "maple")[0])
        status = "ok" if (real and maple) else "FAIL"
        if not (real and maple):
            failures.append(r)
        print(
            f"  {r.distance_in:>4}in  {r.angle_deg:>5.1f} deg  {r.rpm:>5} rpm   "
            f"real={'pass' if real else 'MISS'}  maple={'pass' if maple else 'MISS'}   {status}"
        )

    if failures:
        print(f"\n{len(failures)} row(s) do not score at slip factor {slip_factor}.")
        print("Re-derive with:  --slip-factor %s --write" % slip_factor)
        return 1
    print(f"\nall {len(rows)} rows score under both models.")
    return 0


def main():
    p = argparse.ArgumentParser(
        description="Derive the shot table from physics for a measured slip factor.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument(
        "--slip-factor",
        type=float,
        required=True,
        help="MEASURED flywheel-to-exit-velocity slip factor "
        "(ShooterConstants.EXIT_VELOCITY_SLIP_FACTOR). Every RPM scales with this.",
    )
    p.add_argument("--min-distance-in", type=int, default=60)
    p.add_argument("--max-distance-in", type=int, default=260)
    p.add_argument("--step-in", type=int, default=10)
    p.add_argument("--angle-step-deg", type=float, default=0.5)
    p.add_argument("--rpm-step", type=float, default=10.0)
    p.add_argument("--json", type=Path, help="also write the rows as JSON")
    p.add_argument("--write", action="store_true", help="patch ShooterMath.java in place")
    p.add_argument("--check", action="store_true", help="verify the committed table, do not derive")
    args = p.parse_args()

    if args.slip_factor <= 0:
        sys.exit("--slip-factor must be positive")

    if args.check:
        sys.exit(check_committed(args.slip_factor))

    distances = list(range(args.min_distance_in, args.max_distance_in + 1, args.step_in))
    print(
        f"deriving {len(distances)} rows, slip factor {args.slip_factor}, "
        f"hood {MIN_HOOD_DEG}-{MAX_HOOD_DEG} deg\n",
        file=sys.stderr,
    )

    rows = []
    for d in distances:
        row = solve_distance(d, args.slip_factor, args.angle_step_deg, args.rpm_step, True)
        if row is not None:
            rows.append(row)

    if not rows:
        sys.exit("no distance produced a shot that scores under both models")

    unreachable = set(distances) - {r.distance_in for r in rows}
    if unreachable:
        print(
            f"\nWARNING: no dual-model solution at {sorted(unreachable)} in -- "
            "omitted from the table.",
            file=sys.stderr,
        )

    print(render_java(rows))
    print(
        f"\n// range {rows[0].distance_in}..{rows[-1].distance_in} in, "
        f"margin {min(r.margin_rpm for r in rows)}..{max(r.margin_rpm for r in rows)} RPM"
    )

    if args.json:
        args.json.write_text(json.dumps([asdict(r) for r in rows], indent=1))
        print(f"wrote {args.json}", file=sys.stderr)
    if args.write:
        write_into_shooter_math(rows)


if __name__ == "__main__":
    main()
