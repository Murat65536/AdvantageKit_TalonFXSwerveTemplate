#!/usr/bin/env python3
"""Analyze a trajectory-tracking capture from nt_capture.py.

Reports tracking-error statistics, a time-bucketed table, per-wheel speed reconstruction
(vs the drivetrain limit), and -- aligned against the planned Choreo trajectory -- whether
the error is driven by velocity capability, acceleration, or controller oscillation.

Usage:
    ~/.cache/ntcap-venv/bin/python scripts/nt_analyze.py [trajectory_name]

trajectory_name defaults to "Test" (reads src/main/deploy/choreo/<name>.traj).
"""
import json, math, os, sys

# --- robot geometry / limits (from TunerConstants and choreo.chor) ---
MODULE_HALF = 0.29877  # module x/y offset from center, m (11.7625 in)
WHEEL_SPEED_LIMIT = 4.83  # kSpeedAt12Volts, m/s
MODULES = [(MODULE_HALF, MODULE_HALF), (MODULE_HALF, -MODULE_HALF),
           (-MODULE_HALF, MODULE_HALF), (-MODULE_HALF, -MODULE_HALF)]

CAP = os.path.expanduser("~/.cache/ntcap/capture.json")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
traj_name = sys.argv[1] if len(sys.argv) > 1 else "Test"
TRAJ = os.path.join(ROOT, "src/main/deploy/choreo", traj_name + ".traj")

cap = json.load(open(CAP))


def srt(key):
    return sorted(cap[key], key=lambda p: p[0])


def interp(xs, ys, x):
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    lo, hi = 0, len(xs) - 1
    while hi - lo > 1:
        m = (lo + hi) // 2
        if xs[m] <= x:
            lo = m
        else:
            hi = m
    f = (x - xs[lo]) / (xs[hi] - xs[lo]) if xs[hi] > xs[lo] else 0
    return ys[lo] + f * (ys[hi] - ys[lo])


def corr(a, b):
    n = len(a)
    ma, mb = sum(a) / n, sum(b) / n
    sa = math.sqrt(sum((x - ma) ** 2 for x in a))
    sb = math.sqrt(sum((y - mb) ** 2 for y in b))
    return float("nan") if sa * sb == 0 else sum((x - ma) * (y - mb) for x, y in zip(a, b)) / (sa * sb)


def max_wheel(vx, vy, omega):
    return max(math.hypot(vx - omega * my, vy + omega * mx) for mx, my in MODULES)


err = srt("errTrans")
errh = srt("errHeading")
meas = srt("measured")
setp = srt("setpoints")
t0 = err[0][0]
T = lambda us: (us - t0) / 1e6
dur = T(err[-1][0])
evals = [v for _, v in err]

print("=== TRACKING ERROR (Choreo/ErrorTranslation) ===")
print("  duration:   %.2f s   (%d samples)" % (dur, len(err)))
print("  peak:       %.3f m" % max(evals))
print("  mean:       %.3f m" % (sum(evals) / len(evals)))
print("  at start:   %.3f m" % evals[0])
print("  at end:     %.3f m" % evals[-1])
print("  final heading error: %.2f deg" % math.degrees(errh[-1][1]))

# --- per-wheel speed reconstruction ---
print("\n=== WHEEL SPEEDS (reconstructed from chassis vx,vy,omega) ===")
for key in ("setpoints", "measured"):
    rows = [v for _, v in cap[key]]
    w = [max_wheel(*v) for v in rows]
    over = sum(1 for x in w if x > WHEEL_SPEED_LIMIT + 0.05)
    print("  %-10s max wheel speed: mean %.2f  peak %.2f m/s   (limit %.2f, over: %d/%d)"
          % (key, sum(w) / len(w), max(w), WHEEL_SPEED_LIMIT, over, len(w)))

# --- trajectory-aligned analysis ---
if not os.path.exists(TRAJ):
    print("\n(trajectory file %s not found -- skipping aligned analysis)" % TRAJ)
    sys.exit(0)
tj = json.load(open(TRAJ))
samps = (tj.get("trajectory") or tj)["samples"]
tt = [s["t"] for s in samps]
ispd = [math.hypot(s["vx"], s["vy"]) for s in samps]
iacc = [math.hypot(s["ax"], s["ay"]) for s in samps]
mt = [T(t) for t, _ in meas]
mv = [math.hypot(v[0], v[1]) for _, v in meas]

et = [T(t) for t, _ in err]
ev = [v for _, v in err]
isp = [interp(tt, ispd, t) for t in et]
iac = [interp(tt, iacc, t) for t in et]
ms = [interp(mt, mv, t) for t in et]
deficit = [a - b for a, b in zip(isp, ms)]

print("\n=== TRAJECTORY-ALIGNED (%s.traj) ===" % traj_name)
print("  planned: peak speed %.2f m/s, peak accel %.2f m/s2" % (max(ispd), max(iacc)))
print("  mean planned speed %.2f / mean measured speed %.2f / mean deficit %.2f m/s"
      % (sum(isp) / len(isp), sum(ms) / len(ms), sum(deficit) / len(deficit)))
print("  corr(error, planned speed):        %+.2f" % corr(ev, isp))
print("  corr(error, planned accel):        %+.2f" % corr(ev, iac))
print("  corr(measured speed, planned speed): %+.2f" % corr(ms, isp))
print("  corr(speed deficit, planned accel): %+.2f" % corr(deficit, iac))

print("\n  trajT(s)   plannedSpd  measSpd  deficit  plannedAcc   error")
NB = 8
for b in range(NB):
    a, z = b / NB * dur, (b + 1) / NB * dur
    idx = [i for i, t in enumerate(et) if a <= t < z]
    if not idx:
        continue
    avg = lambda xs: sum(xs[i] for i in idx) / len(idx)
    print("  %4.1f-%4.1f  %9.2f  %7.2f  %7.2f  %9.2f  %7.3f"
          % (a, z, avg(isp), avg(ms), avg(deficit), avg(iac), avg(ev)))
