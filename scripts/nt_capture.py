#!/usr/bin/env python3
"""Capture AdvantageKit NT4 outputs from the running simulation for trajectory-tracking analysis.

Connects an NT4 client to the sim's NetworkTables server, waits for a Choreo auto to start
(Choreo/ErrorTranslation begins publishing), records until it ends (no error update for
~2.5 s), and writes the samples to JSON for nt_analyze.py.

Usage:
    ~/.cache/ntcap-venv/bin/python scripts/nt_capture.py [max_wait_seconds]

Requires the pyntcore package (an NT4 client). Create the venv once with:
    python3 -m venv ~/.cache/ntcap-venv && ~/.cache/ntcap-venv/bin/pip install pyntcore
"""
import ntcore, time, json, struct, sys, os

MAX_WAIT = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
IDLE_END = 2.5  # seconds without an error update => auto finished
START_THRESHOLD = 5  # errTrans samples to call it a real run (rejects a stale retained value)
OUT = os.path.expanduser("~/.cache/ntcap/capture.json")

inst = ntcore.NetworkTableInstance.getDefault()
inst.startClient4("ntcap")
inst.setServer("127.0.0.1")

P = "/AdvantageKit/RealOutputs/"
double_topics = {
    "errTrans": P + "Choreo/ErrorTranslation",
    "errX": P + "Choreo/ErrorX",
    "errY": P + "Choreo/ErrorY",
    "errHeading": P + "Choreo/ErrorHeading",
    "governorFactor": P + "Choreo/GovernorFactor",
    "governedTime": P + "Choreo/GovernedTime",
}
raw_topics = {  # (path, struct type) -- ChassisSpeeds / Pose2d are 3 little-endian doubles each
    "measured": (P + "SwerveChassisSpeeds/Measured", "struct:ChassisSpeeds"),
    "setpoints": (P + "SwerveChassisSpeeds/Setpoints", "struct:ChassisSpeeds"),
    "robot": (P + "Odometry/Robot", "struct:Pose2d"),
    "trajSetpoint": (P + "Odometry/TrajectorySetpoint", "struct:Pose2d"),
}
opts = ntcore.PubSubOptions(sendAll=True, keepDuplicates=True)
dsubs = {k: inst.getDoubleTopic(t).subscribe(float("nan"), opts) for k, t in double_topics.items()}
rsubs = {k: inst.getRawTopic(t).subscribe(ts, b"", opts) for k, (t, ts) in raw_topics.items()}
data = {k: [] for k in list(double_topics) + list(raw_topics)}


def dec(b):
    n = len(b) // 8
    return list(struct.unpack("<%dd" % n, bytes(b)[: n * 8])) if n else []


for _ in range(120):
    if inst.isConnected():
        break
    time.sleep(0.05)
time.sleep(1.5)  # let retained values arrive
for s in list(dsubs.values()) + list(rsubs.values()):  # drain stale retained values
    s.readQueue()
print("READY -- connected:", inst.isConnected(), "-- run a Choreo auto now", flush=True)

t0 = time.time()
started = False
last_err = None
while True:
    now = time.time()
    for k, s in dsubs.items():
        for ev in s.readQueue():
            data[k].append([ev.time, ev.value])
            if k == "errTrans":
                last_err = now
    for k, s in rsubs.items():
        for ev in s.readQueue():
            data[k].append([ev.time, dec(ev.value)])
    if not started and len(data["errTrans"]) >= START_THRESHOLD:
        started = True
        print("  auto detected -- recording", flush=True)
    if not started and now - t0 > MAX_WAIT:
        print("no auto seen in %.0fs -- exiting" % MAX_WAIT, flush=True)
        break
    if started and last_err and now - last_err > IDLE_END:
        print("  auto finished", flush=True)
        break
    time.sleep(0.005)

os.makedirs(os.path.dirname(OUT), exist_ok=True)
json.dump(data, open(OUT, "w"))
print("done. samples:", {k: len(v) for k, v in data.items()})
print("written:", OUT)
