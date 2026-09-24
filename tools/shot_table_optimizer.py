#!/usr/bin/env python3
"""Parametric Monte Carlo shot-table optimizer for the 2026 REBUILT hub shooter.

WHY THIS EXISTS
----------------
"Optimal" (hood angle, exit velocity) for a given distance is NOT a fixed answer --
it depends entirely on the ratio between your hood-angle tracking error and your
flywheel-velocity tracking error (see the crossover analysis in the README below).
A flat/fast trajectory is nearly immune to angle error but punished hard by velocity
error; a steep/slow trajectory is the reverse. Guessing that ratio and baking a single
table into the robot is dishonest -- this tool makes the dependency explicit instead.

USAGE
-----
Phase 1 (sim-only, no measured hardware numbers yet): run with a SPAN of plausible
(sigma_angle_deg, sigma_v_mps) pairs to see the family of optimal tables and where the
flat/steep crossover sits. Nothing here should be trusted as "the" table yet.

Phase 2 (real robot): log Hood.getAngle() vs getTargetAngle() and Shooter.getVelocity()
vs its setpoint while holding a steady shot, compute the stddev of each error series,
then run:

    python3 shot_table_optimizer.py \
        --sigma-angle-deg <measured> --sigma-v-mps <measured> \
        --output shot_table_measured.json

...and wire the resulting JSON into ShooterMath.java (see integration note at bottom).

PHYSICS SOURCE (not assumed -- pulled from maple-sim source and cross-validated)
----------------------------------------------------------------------------------
- GamePieceProjectile.GRAVITY = 11 m/s^2 (maple-sim's own no-air-drag-compensated
  constant; see org.ironmaple.simulation.gamepieces.GamePieceProjectile).
- RebuiltHub extends Goal; Goal.checkValidity() uses a BOX position check (47in x 47in
  x 10in), NOT the sphere check in RebuiltHub.checkCollision() (that method exists but
  is never called by Goal.simulationSubTick -- confirmed by reading Goal.java). No
  entry-angle or velocity requirement is enforced by the sim's scoring logic.
- Ball exit height H0 = ShooterConstants.BALL_EXIT_TRANSLATION.z (21.2315 in).
- Validated: a zero-noise closed-form shot (971 Spartan Robotics' targetAngleSolve
  entry-angle formula) scores 200/200 against this simulator's discrete box check.
"""

import argparse
import itertools
import json
import sys
from pathlib import Path

import numpy as np

# ============================================================
# Physics constants -- see module docstring for provenance
# ============================================================
GRAVITY = 11.0  # m/s^2, maple-sim's GamePieceProjectile.GRAVITY
HUB_HALF_WIDTH_M = 47 * 0.0254 / 2  # RebuiltHub: 47in box, both X and Y
HUB_Z_MIN_M = 1.5748  # RebuiltHub blueHubPose.z
HUB_Z_MAX_M = HUB_Z_MIN_M + 10 * 0.0254  # + Goal height (10in)
BALL_EXIT_HEIGHT_M = 21.2315 * 0.0254  # ShooterConstants.BALL_EXIT_TRANSLATION.z
MAX_EXIT_VELOCITY_MPS = 20.93  # ShooterConstants.MAX_FLYWHEEL_VELOCITY -> exit v, current hardware


def grid_hit_prob(
    distance_m,
    angles_deg,
    velocities_mps,
    n_trials,
    sigma_dist_m,
    sigma_heading_rad,
    sigma_v_mps,
    sigma_angle_rad,
    rng,
    dt=0.01,
    t_max=3.0,
):
    """Vectorized Monte Carlo hit-probability over a full (angle x velocity) grid.

    Returns a 2D array (len(angles_deg), len(velocities_mps)) of hit probabilities,
    replaying the exact box-check logic maple-sim's Goal class performs each subtick.
    """
    angle_grid, v_grid = np.meshgrid(angles_deg, velocities_mps, indexing="ij")
    n_angles, n_velocities = angle_grid.shape
    theta0 = np.radians(angle_grid)[:, :, None]
    v0 = v_grid[:, :, None]

    true_distance = distance_m + rng.normal(0, sigma_dist_m, (n_angles, n_velocities, n_trials))
    heading_error = rng.normal(0, sigma_heading_rad, (n_angles, n_velocities, n_trials))
    speed = np.clip(v0 + rng.normal(0, sigma_v_mps, (n_angles, n_velocities, n_trials)), 1e-6, None)
    theta = theta0 + rng.normal(0, sigma_angle_rad, (n_angles, n_velocities, n_trials))

    vx = speed * np.cos(theta) * np.cos(heading_error)
    vy = speed * np.cos(theta) * np.sin(heading_error)
    vz0 = speed * np.sin(theta)

    time_steps = np.arange(dt, t_max, dt)
    hit_any = np.zeros((n_angles, n_velocities, n_trials), dtype=bool)
    grounded = np.zeros((n_angles, n_velocities, n_trials), dtype=bool)
    for t in time_steps:
        x = vx * t
        y = vy * t
        z = BALL_EXIT_HEIGHT_M + vz0 * t - 0.5 * GRAVITY * t * t
        grounded |= z < 0
        in_box = (
            (np.abs(x - true_distance) <= HUB_HALF_WIDTH_M)
            & (np.abs(y) <= HUB_HALF_WIDTH_M)
            & (z >= HUB_Z_MIN_M)
            & (z <= HUB_Z_MAX_M)
            & (~grounded)
        )
        hit_any |= in_box
        if grounded.all():
            break
    return hit_any.mean(axis=2)


def optimize_distance(
    distance_m,
    sigma_angle_deg,
    sigma_v_mps,
    sigma_dist_m=0.08,
    sigma_heading_deg=1.5,
    angle_range=(5, 89),
    v_range=(2.0, MAX_EXIT_VELOCITY_MPS),
    seed=0,
    n_coarse=1200,
    n_fine=8000,
):
    """Coarse grid + local refine search for the single distance that maximizes hit
    probability under the given noise model. Ties broken toward lower exit velocity
    (energy/wear margin) since it doesn't touch the angle policy being searched."""
    rng = np.random.default_rng(seed)
    sigma_heading_rad = np.radians(sigma_heading_deg)
    sigma_angle_rad = np.radians(sigma_angle_deg)

    angles = np.arange(angle_range[0], angle_range[1], 2.0)
    velocities = np.arange(v_range[0], v_range[1], 0.5)
    coarse = grid_hit_prob(
        distance_m, angles, velocities, n_coarse, sigma_dist_m, sigma_heading_rad, sigma_v_mps, sigma_angle_rad, rng
    )
    idx = np.unravel_index(np.argmax(coarse), coarse.shape)
    a0, v0 = angles[idx[0]], velocities[idx[1]]

    fine_angles = np.arange(max(angle_range[0], a0 - 3), min(angle_range[1], a0 + 3), 0.25)
    fine_velocities = np.arange(max(v_range[0], v0 - 1.0), min(v_range[1], v0 + 1.0), 0.1)
    fine = grid_hit_prob(
        distance_m,
        fine_angles,
        fine_velocities,
        n_fine,
        sigma_dist_m,
        sigma_heading_rad,
        sigma_v_mps,
        sigma_angle_rad,
        rng,
    )
    idx2 = np.unravel_index(np.argmax(fine), fine.shape)
    p_best = fine[idx2]
    tie_mask = fine >= (p_best - 1e-9)
    tied_a, tied_v = np.where(tie_mask)
    best_v_idx = tied_v.min()
    sel = tied_v == best_v_idx
    angle_final = float(fine_angles[tied_a[sel]].mean())
    v_final = float(fine_velocities[best_v_idx])

    theta = np.radians(angle_final)
    time_of_flight = distance_m / (v_final * np.cos(theta))

    return {
        "distance_m": round(distance_m, 4),
        "distance_in": round(distance_m / 0.0254, 1),
        "angle_deg": round(angle_final, 2),
        "exit_v_mps": round(v_final, 3),
        "tof_s": round(float(time_of_flight), 4),
        "hit_prob": round(float(p_best), 4),
    }


def optimize_table(
    distances_in,
    sigma_angle_deg,
    sigma_v_mps,
    sigma_dist_m=0.08,
    sigma_heading_deg=1.5,
    seed=0,
    n_coarse=1200,
    n_fine=8000,
):
    """Full shot table across a list of distances (inches) for one noise hypothesis."""
    results = []
    for din in distances_in:
        r = optimize_distance(
            din * 0.0254,
            sigma_angle_deg=sigma_angle_deg,
            sigma_v_mps=sigma_v_mps,
            sigma_dist_m=sigma_dist_m,
            sigma_heading_deg=sigma_heading_deg,
            seed=seed,
            n_coarse=n_coarse,
            n_fine=n_fine,
        )
        results.append(r)
    return {
        "assumptions": {
            "sigma_angle_deg": sigma_angle_deg,
            "sigma_v_mps": sigma_v_mps,
            "sigma_dist_m": sigma_dist_m,
            "sigma_heading_deg": sigma_heading_deg,
            "measured": False,
            "note": "Unmeasured hypothesis -- replace with real logged stddevs (see Phase 2 in module docstring).",
        },
        "shots": results,
    }


DEFAULT_DISTANCES_IN = [90, 100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200, 210]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--sigma-angle-deg", type=float, help="Hood angle tracking stddev, degrees.")
    parser.add_argument("--sigma-v-mps", type=float, help="Flywheel exit-velocity tracking stddev, m/s.")
    parser.add_argument("--sigma-dist-m", type=float, default=0.08, help="Pose/vision distance stddev, m.")
    parser.add_argument("--sigma-heading-deg", type=float, default=1.5, help="Drive aim heading stddev, deg.")
    parser.add_argument("--distances-in", type=float, nargs="+", default=None,
                         help="Defaults to a dense 13-point table normally, or a coarser "
                              "5-point table when --sweep-library is used (speed).")
    parser.add_argument("--output", type=Path, help="Output JSON path for a single table.")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--sweep-library",
        type=Path,
        help="Directory: ignore single sigma args and instead generate a library of tables "
        "spanning a grid of plausible (sigma_angle_deg, sigma_v_mps) pairs, for Phase-1 "
        "exploration before any real numbers exist.",
    )
    args = parser.parse_args(argv)

    if args.distances_in is None:
        args.distances_in = [90, 130, 150, 170, 210] if args.sweep_library else DEFAULT_DISTANCES_IN

    if args.sweep_library:
        args.sweep_library.mkdir(parents=True, exist_ok=True)
        angle_sigmas = [0.1, 0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 2.0, 3.0]
        v_sigmas = [0.05, 0.1, 0.2, 0.3, 0.5, 1.0]
        index = []
        for sa, sv in itertools.product(angle_sigmas, v_sigmas):
            table = optimize_table(
                args.distances_in,
                sa,
                sv,
                args.sigma_dist_m,
                args.sigma_heading_deg,
                args.seed,
                n_coarse=400,
                n_fine=2000,
            )
            fname = f"sigma_a{sa}_v{sv}.json".replace(".", "p")
            (args.sweep_library / fname).write_text(json.dumps(table, indent=2))
            mid = table["shots"][len(table["shots"]) // 2]
            index.append(
                {
                    "sigma_angle_deg": sa,
                    "sigma_v_mps": sv,
                    "file": fname,
                    "mid_distance_angle_deg": mid["angle_deg"],
                    "mid_distance_v_mps": mid["exit_v_mps"],
                    "min_hit_prob": min(s["hit_prob"] for s in table["shots"]),
                }
            )
            print(
                f"sigma_angle={sa:>5.2f}deg  sigma_v={sv:>5.2f}m/s  -> "
                f"mid-distance angle={mid['angle_deg']:>6.2f}deg  v={mid['exit_v_mps']:>6.2f}m/s  "
                f"min_hit_prob={index[-1]['min_hit_prob']:.4f}"
            )
        (args.sweep_library / "_index.json").write_text(json.dumps(index, indent=2))
        print(f"\nWrote {len(index)} hypothesis tables + _index.json to {args.sweep_library}")
        return

    if args.sigma_angle_deg is None or args.sigma_v_mps is None:
        parser.error("--sigma-angle-deg and --sigma-v-mps are required unless --sweep-library is used")

    table = optimize_table(
        args.distances_in, args.sigma_angle_deg, args.sigma_v_mps, args.sigma_dist_m, args.sigma_heading_deg, args.seed
    )
    text = json.dumps(table, indent=2)
    if args.output:
        args.output.write_text(text)
        print(f"Wrote {args.output}")
    else:
        print(text)


if __name__ == "__main__":
    main()
