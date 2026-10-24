#!/usr/bin/env python3
"""Generate src/main/deploy/elastic-layout.json, the Elastic dashboard for Wescavator.

Load it in Elastic with File -> Load Layout From Robot, mode "Full Reload" (the robot
serves its deploy folder on port 5800, see Robot.java). Regenerate after changing any
key in Dashboard.java or SystemsCheck.java, then deploy:

    python3 tools/gen_elastic_layout.py

ElasticLayoutContractTest checks every widget has a published topic, sits on the grid,
and does not overlap another.

Format: Elastic v2026.1.2 (DashboardPage.toJson, TabGridModel, NTWidgetContainerModel,
https://github.com/Gold872/elastic-dashboard/tree/v2026.1.2/lib). x, y, width and height
are pixels and MUST be JSON floats: Elastic reads them with tryCast<double>, which
rejects integers and silently drops every widget into a 1x1 cell at the origin.
"""

import json
from pathlib import Path

GRID = 128
PERIOD = 0.06  # Elastic's default update period
ROBOT_WIDTH_M = 0.886  # DriveConstants.BUMPER_WIDTH_METERS

GREEN, RED = 0xFF4CAF50, 0xFFF44336

SD = "/SmartDashboard/"
MATCH = SD + "Match/"
PIT = SD + "Pit/"


def widget(title, wtype, topic, col, row, w, h, **props):
    return {
        "title": title,
        "x": float(col * GRID),
        "y": float(row * GRID),
        "width": float(w * GRID),
        "height": float(h * GRID),
        "type": wtype,
        "properties": {"topic": topic, "period": PERIOD, **props},
    }


def light(title, topic, col, row, w=2, h=1, on=GREEN, off=RED):
    return widget(
        title, "Boolean Box", topic, col, row, w, h,
        data_type="boolean", true_color=on, false_color=off,
    )


def battery(col, row):
    return widget(
        "Battery", "Voltage View", SD + "Battery", col, row, 2, 1,
        data_type="double", min_value=4.0, max_value=13.0, divisions=5,
        inverted=False, orientation="horizontal",
    )


# Match: only what is worth a glance. Before the match: auto, placement, battery.
# During it: is the hub scoring, can I shoot from here, did anything disconnect, and
# manual mode (operator-set shot, from the 2026RobotProject). Match time and FMS state
# are already on the Driver Station.
#
#   col  0 1 2 3     | 4 5 6         | 7 8 9
#   row0 Auto        | Start Pose OK | Batt. | Alerts
#   row1 Field       | Hub Active    |       |
#   row2             | In Range      |       |
#   row3 Manual Mode | Manual RPM | Manual Hood | Retract Delay (s) | Shot RPM Offset
match_tab = [
    widget("Auto", "ComboBox Chooser", SD + "Auto Choices", 0, 0, 3, 1, sort_options=False),
    light("Start Pose OK", MATCH + "Start Pose OK", 3, 0),
    battery(5, 0),
    widget(
        "Field", "Field", MATCH + "Field", 0, 1, 4, 2,
        field_game="Rebuilt", robot_width=ROBOT_WIDTH_M, robot_length=ROBOT_WIDTH_M,
        show_other_objects=True, show_trajectories=False,
    ),
    light("Hub Active", MATCH + "Hub Active", 4, 1, w=3),
    light("In Range", MATCH + "In Range", 4, 2, w=3),
    widget("Alerts", "Alerts", SD + "Alerts", 7, 0, 3, 3),
    widget(
        "Manual Mode", "Toggle Switch", MATCH + "Manual Mode", 0, 3, 2, 1, data_type="boolean",
    ),
    widget(
        "Manual RPM", "Text Display", MATCH + "Manual RPM", 2, 3, 2, 1,
        data_type="double", show_submit_button=False,
    ),
    widget(
        "Manual Hood", "Text Display", MATCH + "Manual Hood", 4, 3, 2, 1,
        data_type="double", show_submit_button=False,
    ),
    # Editable: seconds to wait after feeding starts before the hopper pulls in.
    widget(
        "Retract Delay (s)", "Text Display", MATCH + "Retract Delay", 6, 3, 2, 1,
        data_type="double", show_submit_button=True,
    ),
    # Editable: RPM added to every automatic shot (+ farther, - shorter), for field tuning.
    widget(
        "Shot RPM Offset", "Text Display", MATCH + "Shot RPM Offset", 8, 3, 2, 1,
        data_type="double", show_submit_button=True,
    ),
]

# Pit: run the check, read the result; zero the hood after any hood work. Kept within
# the Match tab's 10 x 4 footprint so it fits a laptop screen.
#
#   col  0 1 | 2 3      | 4 5    | 6 7     | 8 9
#   row0 Run | Baseline | Result | Battery | Hood Angle
#   row1-3   Systems Check (one row per step)  | Jog Steeper
#   row2                                       | Jog Shallower
#   row3                                       | Zero Hood
pit_tab = [
    widget("Run Systems Check", "Command", PIT + "Run Systems Check", 0, 0, 2, 1, show_type=False),
    widget("Save Baseline", "Command", PIT + "Save Baseline", 2, 0, 2, 1, show_type=False),
    widget(
        "Result", "Text Display", PIT + "Result", 4, 0, 2, 1,
        data_type="string", show_submit_button=False,
    ),
    battery(6, 0),
    widget("Systems Check", "Alerts", SD + "Systems Check", 0, 1, 8, 3),
    widget(
        "Hood Angle", "Text Display", PIT + "Hood Angle", 8, 0, 2, 1,
        data_type="double", show_submit_button=False,
    ),
    widget("Hood Jog Steeper", "Command", PIT + "Hood Jog Steeper", 8, 1, 2, 1, show_type=False),
    widget(
        "Hood Jog Shallower", "Command", PIT + "Hood Jog Shallower", 8, 2, 2, 1, show_type=False,
    ),
    widget(
        "Zero Hood (at steep stop)", "Command", PIT + "Zero Hood", 8, 3, 2, 1,
        show_type=False,
    ),
]

# Cameras: both PhotonVision streams, as the 2026RobotProject's Drive Team tab showed them.
# PhotonVision (on the coprocessor) publishes these, not the robot code. Port 1182 is the first
# camera configured in PhotonVision and 1184 the second; the "Output" stream has the AprilTag
# overlay. If a stream stays blank, right-click it in Elastic and pick the right one.
CAMERA_PUBLISHER = "/CameraPublisher/"
cameras_tab = [
    widget(
        "Camera 1", "Camera Stream",
        CAMERA_PUBLISHER + "photonvision_Port_1182_Output_MJPEG_Server", 0, 0, 5, 4,
        rotation_turns=0,
    ),
    widget(
        "Camera 2", "Camera Stream",
        CAMERA_PUBLISHER + "photonvision_Port_1184_Output_MJPEG_Server", 5, 0, 5, 4,
        rotation_turns=0,
    ),
]

layout = {
    "version": 1.0,
    "grid_size": GRID,
    "tabs": [
        {"name": "Match", "grid_layout": {"layouts": [], "containers": match_tab}},
        {"name": "Pit", "grid_layout": {"layouts": [], "containers": pit_tab}},
        {"name": "Cameras", "grid_layout": {"layouts": [], "containers": cameras_tab}},
    ],
}

out = Path(__file__).resolve().parent.parent / "src/main/deploy/elastic-layout.json"
out.write_text(json.dumps(layout, indent=2) + "\n")
print(f"wrote {out} ({sum(len(t['grid_layout']['containers']) for t in layout['tabs'])} widgets)")
