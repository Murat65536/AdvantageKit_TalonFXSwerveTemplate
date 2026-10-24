# Wescavator dashboard and pit check

## Loading the dashboard

The robot code ships an Elastic layout, `src/main/deploy/elastic-layout.json`, and serves it on port 5800.

1. Open Elastic and connect to the robot (team 1038).
2. Choose **File → Load Layout From Robot**, pick `elastic-layout.json`, and choose **Full Reload**. That clears out any older copy.
3. Save the layout. You only need to do this once per laptop, or again after the layout changes.

To change the layout, edit `tools/gen_elastic_layout.py`, run it, and deploy. `ElasticLayoutContractTest` checks that every widget has a live value, sits on the grid, and doesn't overlap another widget.

## Controls

Two Xbox controllers, the same layout as the 2026RobotProject. The **driver** is on port 0 and the **operator** on port 1. Check the port order in the Driver Station's USB tab. Both controllers rumble while the driver is aiming, and in the last 3 s before your hub activates.

**Driver**

| Button | Does |
|---|---|
| Left stick / right stick X | Drive (field-relative) / turn. Slows down automatically while driving onto a bump. |
| Left trigger (hold) | Aim at the hub and spin up the shot: flywheel, kicker and hood. You can keep driving with the left stick. Does nothing in manual mode. |
| Left bumper (hold) | Snap the heading straight for the trench or a bump |
| X (hold) | Lock the wheels in an X |
| Start | Reset field heading. Point the robot straight away from your driver station first. |

**Operator**

| Button | Automatic mode | Manual mode |
|---|---|---|
| Right trigger (hold) | Feed the shot. Fuel only goes once it's ready: the driver is holding left trigger, the robot is aimed, the flywheel and hood are on target, and your hub will count it. | Spin the flywheel at the manual RPM (with the kicker). Feed with B. |
| Right bumper / left bumper (hold) | Intake (once the hopper is out) / outtake | Same |
| A / Y | Hopper out / in | Same |
| X (hold) | | Kicker |
| B (hold) | | Feeder and indexer |
| D-pad up / down | Manual RPM +50 / −50 (2000–6750) | Same |
| D-pad right / left | Manual hood +2° / −2° (55–76°; never past the 55° soft limit) | Same |
| Start | Reset the manual values to 2900 RPM and 70° | Same |

In manual mode, the hood holds the manual angle for all of teleop.

## Match tab

| Widget | Look at it when |
|---|---|
| Auto, Start Pose OK, Battery | Before the match. Red Start Pose means the robot is more than 5 cm or 3° off; the Field view shows which way to move it. |
| Hub Active | Green means your hub is scoring right now. |
| In Range | Green means you're close enough to shoot. |
| Manual Mode, Manual RPM, Manual Hood | Flip **Manual Mode** on when the automatic shot can't be trusted (bad vision or pose). The operator then sets the flywheel speed and hood angle with the D-pad, and the two readouts show them. |
| Alerts | Something disconnected. |

Match time and FMS state aren't shown here, because the Driver Station already has them.

## Cameras tab

Both PhotonVision camera streams, with the AprilTag overlay. They come straight from the coprocessor, so they work even if the robot code isn't running. If a stream stays blank, right-click it in Elastic and pick the camera from the list.

## Autos

| Auto | Start | What it does | Sim score |
|---|---|---|---|
| Left Trench (2 volleys) | Under the left trench, facing the neutral zone | Two sweeps with a volley after each | 48–68 |
| Right Trench (2 volleys) | Left Trench mirrored to the right trench | Same as Left Trench | 62–74 |
| Center Preload (8 fuel) | Rear bumper against the middle of your hub's alliance-side face | Backs up 1.41 m and shoots all 8 fuel | 8/8 |
| Center Depot (preload + depot) | Same as Center Preload | Drives to the tower end of the depot, sweeps along it next to the wall, leaves past the far end, then shoots the preload and the depot fuel from beside the hub. Not yet run on the robot: watch the tower on the way in | Not measured: the sim robot clips its tower model on the way in |

## Pit systems check (Pit tab)

Run it every time you're back from a match. It takes about 5 s.

1. Put the robot on the cart with the wheels off the ground and an empty hopper. The kicker and feeder spin forward, so any fuel left inside gets shot. Enable in **Teleop**.
2. Press **Run Systems Check**. It won't run while connected to the field (FMS), and it stops the moment you disable.

Every step appears in the Systems Check list:

| Status | Meaning |
|---|---|
| FAIL (red) | The device is disconnected, or the mechanism never reached its target. Fix it before the next match. |
| WARN (yellow) | It works, but it's more than 1.5× slower than the saved baseline: binding, a slipping belt, or a weak motor. Look at it. |
| PASS (blue) | Normal. |

**What each step checks:**
- **Gyro and both cameras:** present on the bus.
- **Swerve steer:** each module turns to 90°.
- **Swerve drive:** each wheel spins at 0.24 m/s forward, then reverse, and the encoder has to agree with the direction.
- **Intake, indexer, feeder, kicker:** each reaches 90% of its normal speed.
- **Shooter:** spins up to the 150 in shot speed.
- **Hood:** moves to 55° (the soft limit; the 50° hard stop breaks the hood) and to 76°. The driver's trim is ignored.
- **Extension:** pushes out (the limit switch must release), then pulls in (the limit switch must press).

### Baseline

WARN compares each step against your robot's own healthy times. Set it once:

1. Right after the robot has been checked over and is known good, run the check and make sure nothing FAILs.
2. Press **Save Baseline**. The times are stored in roboRIO Preferences, so they survive reboots and deploys.
3. Save a new baseline after any mechanical change that makes something faster or slower, such as a gearbox swap, a new belt, or new wheels.

It won't save from a run that had a FAIL or was cut short.

## Zeroing the hood (Pit tab)

Do this once. Do it again if the hood's left Spark MAX is replaced, or whenever the hood, its encoder, or its encoder gear has been taken apart or re-meshed. Until the hood is zeroed, the Alerts list shows "Hood encoder not zeroed". The hood still works, but its angles can be wrong, so it can aim badly or drive into a hard stop.

1. Enable in **Teleop**, not on the field. The jog buttons and Zero Hood refuse to run while connected to FMS.
2. Press **Hood Jog Steeper**. Each press drives the hood slowly for 0.2 s, then stops. Keep pressing until it rests against the steep hard stop (fully retracted, the steepest launch angle).
3. Press **Zero Hood**. Within about half a second, **Hood Angle** should read 76. You don't need to disable: the hood holds still while the zero is applied. If the Alerts list shows "Zero Hood failed", the old zero was kept: try again, and tell a programmer if it keeps failing.
4. Jog **shallower** until the hood stops on its own at about **55**. That's the soft limit: the hood is never sent past 55° because driving it to the 50° hard stop breaks it. Look at the hood: there should still be a gap of a few degrees before the shallow hard stop. If it's touching the stop, or the gap looks much bigger than about 5°, the hood's travel doesn't match the code (50–76°), so don't use the hood until it's been looked at.

If a press doesn't move the hood at all, or moves it more than a few degrees, change `HOOD_JOG_VOLTAGE` in `HoodConstants`.

The zero is saved on the hood's left Spark MAX (CAN ID 13), in its flash memory, so it survives power cycles and deploys. It's the same absolute encoder zero offset that REV Hardware Client shows and sets, so you can zero with either one. After zeroing in REV Hardware Client, hold the hood at the steep stop: the Spark should report about 0 there (76° on the dashboard).

To check that the zero survived a power cycle, compare `IO/Hood/SparkZeroOffsetRot` in AdvantageScope before and after. The robot code reads it back from the Spark at every boot.
