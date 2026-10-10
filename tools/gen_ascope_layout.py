#!/usr/bin/env python3
"""Generate the Wescavator AdvantageScope layout.

Every log key is checked against (a) a real sim WPILOG and (b) the Logger.recordOutput calls in
src/main, so the layout can't reference a key the robot doesn't publish. Keys that are only written
while shooting / in auto / during tuning won't be in an idle sim log, which is why (b) exists.

Schema taken from AdvantageScope v26.0.2 source:
  TabType.ts (enum order), *Controller.ts saveState(), *Controller_Config.ts source types.
"""

import glob
import json
import re
import sys
import uuid

REPO = "/home/bob/programs/frc/Wescavator"
LOG_KEYS = "/home/bob/.hermes/profiles/frc/cache/scratch/simlog_keys.tsv"
OUT = f"{REPO}/AdvantageScope Wescavator.json"

# TabType enum order (src/shared/TabType.ts)
DOCS, LINE, FIELD2D, FIELD3D, TABLE, CONSOLE, STATS, VIDEO, JOYSTICKS, SWERVE = range(10)

# GraphColors used by AdvantageScope's line graph
C = ["#2b66a2", "#e5b31b", "#af2437", "#80588e", "#e48b32", "#3b875a", "#ff7da6"]


# --------------------------------------------------------------------------------------------
# Key validation
# --------------------------------------------------------------------------------------------
def known_keys():
    keys = {}
    for line in open(LOG_KEYS):
        name, typ, *_ = line.rstrip("\n").split("\t")
        keys[name] = typ
    src_keys = set()
    for f in glob.glob(f"{REPO}/src/main/java/**/*.java", recursive=True):
        s = open(f).read()
        for m in re.finditer(r'recordOutput\(\s*"([^"]+)"', s):
            src_keys.add("/RealOutputs/" + m.group(1))
        for m in re.finditer(r'String \w*LOG_KEY\s*=\s*"([^"]+)"', s):
            src_keys.add("/RealOutputs/" + m.group(1))
    return keys, src_keys


LOGGED, SOURCE = known_keys()
used = []


def k(key):
    """Register a key; resolve struct sub-fields (e.g. .../Robot/rotation) to their parent."""
    used.append(key)
    return key


def src(type_, key, log_type, **options):
    return {"type": type_, "logKey": k(key), "logType": log_type, "visible": True, "options": options}


def line(key, color, log_type="Number"):
    return src("stepped", key, log_type, color=color, size="normal")


def stripe(key, color, log_type="Boolean"):
    return src("stripes", key, log_type, color=color)


def unit(target=None):
    return {"autoTarget": target, "preset": None}


def graph(title, left=(), right=(), discrete=(), left_unit=None, right_unit=None,
          left_range=None, right_range=None):
    return tab(LINE, title, {
        "leftSources": list(left),
        "rightSources": list(right),
        "discreteSources": list(discrete),
        "leftLockedRange": left_range,
        "rightLockedRange": right_range,
        "leftUnitConversion": unit(left_unit),
        "rightUnitConversion": unit(right_unit),
        "leftFilter": 0,
        "rightFilter": 0,
    })


def tab(type_, title, controller, controls_height=200):
    return {
        "type": type_,
        "title": title,
        "controller": controller,
        "controllerUUID": uuid.uuid4().hex[:32],
        "renderer": None,
        "controlsHeight": controls_height,
    }


RPM = "rotations/minute"
RO = "/RealOutputs"

# --------------------------------------------------------------------------------------------
# Tabs
# --------------------------------------------------------------------------------------------
tabs = [tab(DOCS, "", None, 0)]

# 1. Field: everything spatial in one 3D view.
tabs.append(tab(FIELD3D, "Field", {
    "game": "FRC:2026 Field",
    "sources": [
        src("robot", f"{RO}/Odometry/Robot", "Pose2d", model="Wescavator"),
        # Components attached to the robot model (model_0: Intake/Extension, model_1: Shooter/Hood)
        src("component", f"{RO}/Subsystems/Extension/ComponentPose", "Pose3d"),
        src("component", f"{RO}/Subsystems/Hood/ComponentPose", "Pose3d"),
        # Aim line: robot -> the point the shooter is aiming at (only while shooting).
        src("vision", f"{RO}/Subsystems/Shooter/Target", "Translation2d", color="#ffff00", size="bold"),
        # Vision: lines to seen tags, plus accepted/rejected pose estimates.
        src("vision", f"{RO}/Vision/Summary/TagPoses", "Pose3d[]", color="#00ff00", size="normal"),
        src("ghost", f"{RO}/Vision/Summary/RobotPosesAccepted", "Pose3d[]", model="Wescavator", color="#00ff00"),
        src("ghost", f"{RO}/Vision/Summary/RobotPosesRejected", "Pose3d[]", model="Wescavator", color="#ff0000"),
        # Sim only: where the robot really is (odometry drift = gap between this and the robot).
        src("ghost", f"{RO}/FieldSimulation/RobotGroundTruth", "Pose2d", model="Wescavator", color="#00ffff"),
        # Auto: where Choreo wants the robot right now.
        src("ghost", f"{RO}/Odometry/TrajectorySetpoint", "Pose2d", model="Wescavator", color="#ff8c00"),
        # Fuel on the field, fuel in the robot, and the last shot's arc.
        src("gamePiece", f"{RO}/FieldSimulation/FuelPositions", "Pose3d[]", variant="Fuel"),
        src("gamePiece", f"{RO}/FieldSimulation/HeldFuelPositions", "Pose3d[]", variant="Fuel"),
        src("trajectory", f"{RO}/FieldSimulation/ShooterTrajectory", "Pose3d[]", color="#ff00ff", size="bold"),
    ],
}))

# 2. Shot readiness: the four gates, in the order ShootCommands checks them.
tabs.append(graph(
    "Shot Readiness",
    left=[line(f"{RO}/Shoot/AimErrorDeg", C[2])],
    right=[line(f"{RO}/Subsystems/Shooter/TargetDistanceM", C[0])],
    discrete=[
        stripe(f"{RO}/Shoot/Feeding", "#3b875a"),
        stripe(f"{RO}/Shoot/FlywheelReady", C[1]),
        stripe(f"{RO}/Shoot/HoodReady", C[3]),
        stripe(f"{RO}/Shoot/Aimed", C[2]),
        stripe(f"{RO}/Subsystems/Shooter/TargetInRange", C[0]),
    ],
))

# 3. Flywheel: measured vs setpoint in RPM, current shows each ball's dip.
tabs.append(graph(
    "Flywheel",
    left=[
        line("/IO/Shooter/Velocity", C[0]),
        line("/IO/Shooter/VelocitySetpoint", C[1]),
    ],
    right=[line("/IO/Shooter/CurrentOut", C[2])],
    discrete=[
        stripe(f"{RO}/Subsystems/Shooter/AtSetpoint", C[1]),
        stripe(f"{RO}/Shoot/Feeding", "#3b875a"),
    ],
    left_unit=RPM,
))

# 4. Hood: measured angle (converted rad -> deg) against target, same locked scale on both axes.
tabs.append(graph(
    "Hood",
    left=[line("/IO/Hood/Angle", C[0])],
    right=[
        line(f"{RO}/Subsystems/Hood/TargetAngleDeg", C[1]),
        line(f"{RO}/Subsystems/Hood/TrimDeg", C[3]),
    ],
    discrete=[stripe("/IO/Hood/AtSetpoint", C[1])],
    left_unit="degrees",
    left_range=[45, 80],
    right_range=[45, 80],
))

# 5. Feed path: every roller between intake and flywheel.
tabs.append(graph(
    "Feed Path",
    left=[
        line("/IO/Kicker/Velocity", C[0]),
        line("/IO/Feeder/Velocity", C[1]),
        line("/IO/Indexer/Velocity", C[2]),
        line("/IO/Intake/RollerVelocity", C[3]),
    ],
    right=[
        line("/IO/Kicker/Current", C[0]),
        line("/IO/Feeder/Current", C[1]),
        line("/IO/Indexer/Current", C[2]),
        line("/IO/Intake/RollerCurrentOut", C[3]),
    ],
    discrete=[
        stripe(f"{RO}/Shoot/Feeding", "#3b875a"),
        stripe(f"{RO}/Subsystems/Extension/FullyExtended", C[4]),
        stripe(f"{RO}/FieldSimulation/HeldFuelCount", C[5], "Number"),
    ],
    left_unit=RPM,
))

# 6. Driver feedback: hub shift schedule and what the controller is feeling.
tabs.append(graph(
    "Hub & Rumble",
    left=[
        line(f"{RO}/DriverRumble/Output", C[0]),
        line(f"{RO}/DriverRumble/Aiming", C[1]),
        line(f"{RO}/DriverRumble/HubWarning", C[2]),
    ],
    right=[line(f"{RO}/Hub/SecondsUntilActive", C[3])],
    discrete=[
        stripe(f"{RO}/Hub/Shift", C[3], "String"),
        stripe(f"{RO}/Hub/Active", "#3b875a"),
        stripe(f"{RO}/Hub/ActivatingSoon", C[2]),
    ],
    left_range=[0, 1],
))

# 7. Drive: module vectors (kept from the AdvantageKit template).
tabs.append(tab(SWERVE, "Swerve", {
    "sources": [
        src("states", f"{RO}/SwerveStates/Measured", "SwerveModuleState[]", color="#ff0000", arrangement="0,1,2,3"),
        src("states", f"{RO}/SwerveStates/SetpointsOptimized", "SwerveModuleState[]", color="#00ffff", arrangement="0,1,2,3"),
        src("chassisSpeeds", f"{RO}/SwerveChassisSpeeds/Measured", "ChassisSpeeds", color="#ff0000"),
        src("chassisSpeeds", f"{RO}/SwerveChassisSpeeds/Setpoints", "ChassisSpeeds", color="#00ffff"),
        src("rotation", f"{RO}/Odometry/Robot/rotation", "Rotation2d"),
    ],
    "maxSpeed": 5,
    "sizeX": 0.65,
    "sizeY": 0.65,
    "orientation": 1,
}))

# 8. Auto: Choreo tracking error, with Choreo's own alerts underneath.
tabs.append(graph(
    "Auto Tracking",
    left=[
        line(f"{RO}/Choreo/ErrorX", C[0]),
        line(f"{RO}/Choreo/ErrorY", C[1]),
        line(f"{RO}/Choreo/ErrorTranslation", C[2]),
    ],
    right=[line(f"{RO}/Choreo/ErrorHeading", C[3])],
    discrete=[src("alerts", f"{RO}/Choreo Alerts", "Alerts")],
))

# 9. Tuning: live traces for the two measurement routines. Final numbers land in Console.
tabs.append(graph(
    "Tuning",
    left=[
        line(f"{RO}/Tuning/Dip/InstantRpm", C[0]),
        line(f"{RO}/Tuning/InstantRpmErrorRpm", C[1]),
    ],
    right=[line(f"{RO}/Tuning/InstantAngleErrorDeg", C[2])],
    discrete=[stripe(f"{RO}/Shoot/Feeding", "#3b875a")],
))

# 10. Robot health: every alert source, loop time, and device connection.
tabs.append(graph(
    "Health",
    left=[
        line(f"{RO}/LoggedRobot/FullCycleMS", C[2]),
        line(f"{RO}/LoggedRobot/UserCodeMS", C[0]),
    ],
    discrete=[
        src("alerts", f"{RO}/Alerts", "Alerts"),
        src("alerts", f"{RO}/PhotonAlerts", "Alerts"),
        stripe("/Drive/Gyro/Connected", C[0]),
        stripe("/IO/Shooter/Connected", C[1]),
        stripe("/IO/Hood/EncoderConnected", C[3]),
        stripe("/IO/Kicker/Connected", C[4]),
        stripe("/IO/Feeder/Connected", C[5]),
        stripe("/IO/Indexer/Connected", C[6]),
        stripe("/IO/Intake/RollerConnected", C[2]),
        stripe("/IO/Extension/Connected", C[0]),
        stripe("/Vision/Camera0/Connected", C[1]),
        stripe("/Vision/Camera1/Connected", C[3]),
    ],
    left_range=[0, 25],
))

# 11. Console: where every tuning routine prints its final numbers.
tabs.append(tab(CONSOLE, "Console", f"{RO}/Console", 0))
used.append(f"{RO}/Console")

# --------------------------------------------------------------------------------------------
# Validate
# --------------------------------------------------------------------------------------------
STRUCT_FIELDS = {"/rotation", "/translation"}
missing = []
for key in used:
    base = key
    for suffix in STRUCT_FIELDS:
        if base.endswith(suffix):
            base = base[: -len(suffix)]
    if base in LOGGED:
        continue
    # Alerts groups log as <key>/.type etc.
    if f"{base}/.type" in LOGGED:
        continue
    if base in SOURCE:
        continue
    # Vision/Camera1 exists only as the second PhotonVision camera.
    if re.sub(r"Camera\d", "Camera0", base) in LOGGED:
        continue
    missing.append(key)

if missing:
    print("KEYS NOT PUBLISHED BY THE ROBOT:", *missing, sep="\n  ")
    sys.exit(1)

live = sum(1 for key in used if key in LOGGED or f"{key}/.type" in LOGGED
           or any(key.endswith(s) and key[: -len(s)] in LOGGED for s in STRUCT_FIELDS))
print(f"{len(used)} keys: {live} seen in the idle sim log, "
      f"{len(used) - live} only written while shooting/auto/tuning (verified in source)")
for key in used:
    if not (key in LOGGED or f"{key}/.type" in LOGGED
            or any(key.endswith(s) and key[: -len(s)] in LOGGED for s in STRUCT_FIELDS)):
        print("   event-only:", key)

layout = {
    "hubs": [{
        "x": 100, "y": 60, "width": 1400, "height": 900,
        "state": {
            "sidebar": {"width": 300, "expanded": ["/RealOutputs", "/RealOutputs/Shoot", "/IO"]},
            "tabs": {"selected": 1, "tabs": tabs},
        },
    }],
    "satellites": [],
    "version": "26.0.2",
}
json.dump(layout, open(OUT, "w"), indent=1)
print(f"wrote {OUT} ({len(tabs) - 1} tabs)")
