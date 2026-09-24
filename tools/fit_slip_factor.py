#!/usr/bin/env python3
"""Fit EXIT_VELOCITY_SLIP_FACTOR from real robot measurements.

THE PROBLEM
-----------
Exit speed is modelled as

    v_exit = omega_flywheel * FLYWHEEL_EFFECTIVE_RADIUS * SLIP_FACTOR

FLYWHEEL_EFFECTIVE_RADIUS is a tape-measure number. SLIP_FACTOR is not: it lumps
together compression, surface friction, ball state and dwell time. It is
currently 0.58 -- a guess -- and EVERY RPM in the shot table scales linearly with
it. Measure it once and the whole table follows from physics.

TWO METHODS
-----------
`range` (RECOMMENDED, run this first)
    Fire into open space with no hub. Measure how far the fuel flies before it
    first touches the floor. Range + hood angle + launch height invert to exit
    speed through the real drag model, and slip falls out.

    This is the better measurement because it is INDEPENDENT of hub geometry.
    Our hub center and maple-sim's disagree by 1.1 in (see FieldConstantsTest),
    so any method that scores into the hub entangles slip with that unresolved
    error. Range does not touch the hub at all.

`score`
    Park at a measured distance, hold the table's hood angle, sweep RPM, record
    which RPMs go in, and take the middle of the band that scores. Use this to
    VALIDATE the fit from `range`, not to produce it. If the two disagree, the
    difference is hub geometry or barrel offset -- not slip.

CRITICAL: WHICH RPM TO RECORD
-----------------------------
Record the flywheel's MEASURED RPM at the instant the fuel leaves, not the
setpoint. The flywheel dips when a ball passes through, so the ball departs
slower than commanded. Feeding the setpoint into this script folds that dip into
the slip factor and the fit will not transfer between feed rates.
Use `TuningCommands.captureShotDip` to get that number.

USAGE
-----
  nix-shell -p 'python3.withPackages(ps: [ps.numpy])' --run \\
    "python3 tools/fit_slip_factor.py --self-test"

  # range method: --shot ANGLE_DEG,MEASURED_RPM,RANGE_METERS  (repeatable)
  ... --run "python3 tools/fit_slip_factor.py range \\
        --shot 50,3000,5.10 --shot 50,3400,6.32 --shot 60,3000,4.55"

  # score method: --shot DISTANCE_IN,ANGLE_DEG,MEASURED_RPM
  ... --run "python3 tools/fit_slip_factor.py score \\
        --shot 150,76,3220 --shot 210,66.5,3085"
"""

from __future__ import annotations

import argparse
import math
import sys

try:
    import numpy as np
except ImportError:
    sys.exit(
        "numpy is required.\n"
        "  NixOS: nix-shell -p 'python3.withPackages(ps: [ps.numpy])' "
        '--run "python3 tools/fit_slip_factor.py --self-test"'
    )

IN = 0.0254

# --- Robot geometry (ShooterConstants) ---
LAUNCH_HEIGHT = 21.2315 * IN
FLYWHEEL_RADIUS = 2.0 * IN

# --- 6328 FuelSim real-world physics (util/FuelSim.java) ---
GRAVITY = 9.81
AIR_DENSITY = 1.2041
FUEL_RADIUS = 0.075
FUEL_MASS = 0.448 * 0.45392
DRAG_COF = 0.47
DRAG_FORCE_FACTOR = 0.5 * AIR_DENSITY * DRAG_COF * math.pi * FUEL_RADIUS**2

# --- maple-sim scoring volume (FieldConstants) ---
HUB_Z_MIN = 1.5748
HUB_Z_MAX = HUB_Z_MIN + 10.0 * IN
HUB_HALF = 47.0 * IN / 2.0

DT = 0.002
MAX_FLIGHT_S = 8.0


def rpm_to_exit_velocity(rpm, slip):
    return rpm * 2.0 * math.pi / 60.0 * FLYWHEEL_RADIUS * slip


def floor_range(v, angle_deg, launch_height=LAUNCH_HEIGHT):
    """Horizontal distance travelled before the fuel's underside first touches the floor.

    Integrated with the same quadratic-drag model the shot table is validated
    against, so a slip factor fitted here is consistent with that table.
    """
    ang = math.radians(angle_deg)
    x, z = 0.0, launch_height
    vx, vz = v * math.cos(ang), v * math.sin(ang)
    t = 0.0
    while t < MAX_FLIGHT_S:
        prev_x, prev_z = x, z
        x += vx * DT
        z += vz * DT
        t += DT
        speed = math.hypot(vx, vz)
        vx -= DRAG_FORCE_FACTOR * speed * vx / FUEL_MASS * DT
        vz += (-GRAVITY - DRAG_FORCE_FACTOR * speed * vz / FUEL_MASS) * DT
        if z <= FUEL_RADIUS:
            # Linear interpolation onto the contact plane.
            if prev_z != z:
                frac = (prev_z - FUEL_RADIUS) / (prev_z - z)
                return prev_x + (x - prev_x) * frac
            return x
    return float("nan")


def scores(v, angle_deg, distance_m):
    """True if the shot passes through the maple-sim scoring box while descending."""
    ang = math.radians(angle_deg)
    x, z = 0.0, LAUNCH_HEIGHT
    vx, vz = v * math.cos(ang), v * math.sin(ang)
    t = 0.0
    while t < MAX_FLIGHT_S:
        x += vx * DT
        z += vz * DT
        t += DT
        speed = math.hypot(vx, vz)
        vx -= DRAG_FORCE_FACTOR * speed * vx / FUEL_MASS * DT
        vz += (-GRAVITY - DRAG_FORCE_FACTOR * speed * vz / FUEL_MASS) * DT
        if z < 0.0:
            return False
        if abs(distance_m - x) <= HUB_HALF and HUB_Z_MIN <= z <= HUB_Z_MAX:
            return True
    return False


def solve_slip_from_range(angle_deg, rpm, measured_range, lo=0.05, hi=1.5):
    """Bisect on slip until the modelled floor range matches the measured one."""
    omega = rpm * 2.0 * math.pi / 60.0

    def err(slip):
        return floor_range(omega * FLYWHEEL_RADIUS * slip, angle_deg) - measured_range

    if err(lo) > 0 or err(hi) < 0:
        return None
    for _ in range(200):
        mid = (lo + hi) / 2.0
        if err(mid) < 0:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2.0


def solve_slip_from_score(distance_in, angle_deg, rpm):
    """Find the slip factor whose scoring RPM band is centred on the observed RPM."""
    distance = distance_in * IN
    omega = rpm * 2.0 * math.pi / 60.0
    best = None
    for slip in np.arange(0.20, 1.001, 0.0005):
        v = omega * FLYWHEEL_RADIUS * slip
        if scores(v, angle_deg, distance):
            if best is None:
                best = [slip, slip]
            else:
                best[1] = slip
    if best is None:
        return None
    return (best[0] + best[1]) / 2.0


def report(fits, method):
    if not fits:
        sys.exit(
            "No shot produced a valid fit. Check that the measurements are "
            "physically reachable and that RPM is the MEASURED value, not the setpoint."
        )
    vals = np.array([f[1] for f in fits])
    mean, std = vals.mean(), vals.std(ddof=1) if len(vals) > 1 else 0.0

    print(f"\n{'shot':<34} {'fitted slip':>12}")
    print("-" * 48)
    for label, slip in fits:
        print(f"{label:<34} {slip:>12.4f}")
    print("-" * 48)
    print(f"{'mean':<34} {mean:>12.4f}")
    if len(vals) > 1:
        print(f"{'std dev':<34} {std:>12.4f}")
        spread = vals.max() - vals.min()
        print(f"{'spread':<34} {spread:>12.4f}")
        if spread > 0.05:
            print(
                "\nWARNING: spread > 0.05. The single-slip model does not fit these\n"
                "shots. Most likely causes, in order: (1) you recorded the RPM\n"
                "SETPOINT instead of the measured RPM at ball exit, (2) the fuel is\n"
                "worn or inconsistent, (3) slip genuinely varies with RPM -- in which\n"
                "case a constant factor is the wrong model and the table needs a\n"
                "per-RPM curve."
            )
    print(f"\ncurrent value in ShooterConstants: 0.58")
    print(f"measured:                          {mean:.4f}  ({(mean/0.58-1)*100:+.1f}%)")
    print("\nNext:")
    print(f"  1. set EXIT_VELOCITY_SLIP_FACTOR = {mean:.4f} in ShooterConstants.java")
    print(
        f"  2. nix-shell -p 'python3.withPackages(ps: [ps.numpy])' --run "
        f'"python3 tools/derive_shot_table.py --slip-factor {mean:.4f} --write"'
    )
    print("  3. ./gradlew test")


def self_test():
    """Prove the fitter recovers a slip factor it was never told."""
    print("Self-test: generate shots with a KNOWN slip, then fit it back.\n")
    ok = True
    for truth in (0.42, 0.58, 0.73):
        fits = []
        for angle, rpm in ((50, 3000), (55, 3400), (65, 2800)):
            v = rpm_to_exit_velocity(rpm, truth)
            rng = floor_range(v, angle)
            got = solve_slip_from_range(angle, rpm, rng)
            fits.append(got)
            print(
                f"  truth={truth:.3f}  angle={angle:>2}  rpm={rpm}  "
                f"-> range={rng:.3f} m  -> fitted={got:.5f}"
            )
        err = max(abs(f - truth) for f in fits)
        status = "PASS" if err < 1e-4 else "FAIL"
        if err >= 1e-4:
            ok = False
        print(f"  max error {err:.2e}  [{status}]\n")

    # Round-trip the score method too.
    truth = 0.58
    v = rpm_to_exit_velocity(3220, truth)
    if scores(v, 76.0, 150 * IN):
        got = solve_slip_from_score(150, 76.0, 3220)
        err = abs(got - truth)
        print(f"  score method: truth={truth} fitted={got:.4f} err={err:.4f}")
        if err > 0.02:
            ok = False
            print("  [FAIL] score method off by more than 0.02")
        else:
            print("  [PASS] score method within 0.02 (band-centre method is coarser)")
    else:
        print("  score method: 150in/76deg/3220rpm does not score at truth slip; skipped")

    print("\nSELF-TEST " + ("PASSED" if ok else "FAILED"))
    return 0 if ok else 1


def main():
    p = argparse.ArgumentParser(
        description="Fit EXIT_VELOCITY_SLIP_FACTOR from robot measurements.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument("--self-test", action="store_true", help="verify the fitter, then exit")
    sub = p.add_subparsers(dest="method")

    r = sub.add_parser("range", help="fit from floor-landing distance (recommended)")
    r.add_argument(
        "--shot",
        action="append",
        required=True,
        metavar="ANGLE_DEG,MEASURED_RPM,RANGE_M",
        help="repeatable; use the MEASURED rpm at ball exit, not the setpoint",
    )

    s = sub.add_parser("score", help="fit from the RPM band that scores at a known distance")
    s.add_argument(
        "--shot",
        action="append",
        required=True,
        metavar="DISTANCE_IN,ANGLE_DEG,MEASURED_RPM",
        help="repeatable; MEASURED_RPM is the centre of the band that scored",
    )

    args = p.parse_args()

    if args.self_test:
        sys.exit(self_test())
    if not args.method:
        p.print_help()
        sys.exit(1)

    fits = []
    for raw in args.shot:
        parts = [float(x) for x in raw.split(",")]
        if args.method == "range":
            angle, rpm, rng = parts
            slip = solve_slip_from_range(angle, rpm, rng)
            label = f"{angle:g}deg {rpm:g}rpm -> {rng:g}m"
        else:
            dist, angle, rpm = parts
            slip = solve_slip_from_score(dist, angle, rpm)
            label = f"{dist:g}in {angle:g}deg {rpm:g}rpm"
        if slip is None:
            print(f"  {label}: NO FIT (unreachable)", file=sys.stderr)
            continue
        fits.append((label, slip))

    report(fits, args.method)


if __name__ == "__main__":
    main()
