# Changelog

This documents all uncommitted working-tree changes relative to commit `4619fca`
(`More choreo stuff`), with rationale.

## 2026-05-22

### Code review fixes

A review of the codebase surfaced eight bugs and inconsistencies, fixed here.

- **Steer motors now use their configured motion profile**
  (`ModuleIOTalonFX.java`). The steer config sets MotionMagicExpo parameters, but
  `setTurnPosition()` issued plain `PositionVoltage` / `PositionTorqueCurrentFOC`
  requests, so the profile was dead config. Switched to `MotionMagicExpoVoltage` /
  `MotionMagicExpoTorqueCurrentFOC`.
- **Fixed simulated gyro angular velocity** (`GyroIOSim.java`). `yawVelocityRadPerSec`
  was passed through `Units.degreesToRadians()` on a value already in rad/s — a double
  conversion making it ~57× too small. Now used directly; unused import removed.
- **Extension full-extension detection uses the limit switch only**
  (`Extension.java`, `ExtensionConstants.java`). `isFullyExtended()` had a fallback
  comparing the encoder position to `MAX_POSITION_ROT`, a sim-only constant that does
  not match the real encoder. Dropped the fallback; removed `FULLY_EXTENDED_TOLERANCE_ROT`.
- **Fixed unit mismatch in `Extension.getComponentPose()`**. A value in meters was
  clamped against rotation-unit bounds; the bounds are now converted to meters.
- **Shooter valid-range now matches the lookup table** (`ShooterMath.java`).
  `maxDistanceMeters` was 5.334 m but the time-of-flight table only covers to 4.5 m, so
  far shots silently clamped time-of-flight. Lowered to 4.5 m.
- **Collapsed redundant `ShotSolution` fields** (`ShooterMath.java`, `Shooter.java`).
  `distanceMeters`, `frontEdgeDistanceMeters`, and `rpmLookupDistanceMeters` were always
  assigned the same value; collapsed to one `distanceMeters`, removed the duplicate log
  outputs.
- **Shooter current now sums all four motors** (`ShooterIOReal.java`). `currentOut`
  previously summed only the two top flywheel motors.
- **Removed a no-op `.flatten()`** on `bottomRightMotorConfig` (`ShooterIOReal.java`) —
  a debugging leftover whose return value was discarded.

### Autonomous trajectory tracking

The Choreo trajectory follower tracked poorly in simulation (~0.63 m mean / 1.10 m peak
error, with the drivetrain saturating mid-path). A measurement-driven diagnosis found
several compounding causes, fixed in sequence:

- **Corrected the simulated drive velocity feedforward** (`PhoenixUtil.java`,
  `regulateModuleConstantForSimulation`). The hardcoded `kV = 0.124` was a rotor-unit
  value, but the drive closed loop runs in mechanism (wheel) units — so the feedforward
  was ~6.4× too small and the sim drivetrain only reached ~3 m/s when commanded its
  rated 4.83 m/s. `kV` is now derived from `SpeedAt12Volts` and the wheel radius (≈0.79).
- **Restored the drive static/acceleration feedforward** (`PhoenixUtil.java`).
  `kS` and `kA` were zeroed in simulation; restored to the physical values (0.18, 0.06)
  matching `TunerConstants`. The missing `kA` had caused a heading-tracking lag during
  hard rotation. (This method is `isReal()`-gated — these are simulation-only.)
- **Re-tuned the Choreo translation PD gains** (`Drive.java`). `CHOREO_TRANSLATION_KP`
  10 → 4, `CHOREO_TRANSLATION_KD` 0 → 0.1. `kP = 10` with no damping was over-aggressive:
  it amplified normal tracking error into commanded speeds far past the drivetrain limit,
  saturating the drive and producing an oscillation around the path.
- **Derated the Choreo trajectory model** (`choreo.chor`). `vmax` 4800 → 3900 RPM,
  `tmax` 1.16 → 0.95 N·m. Trajectories were planned at ~100% of robot capability, leaving
  the follower no headroom to correct error. Derating plans trajectories below capability
  so the feedforward fits with room for feedback. Trajectories were regenerated
  (`Test.traj`, `Left_Trench.traj`, and `ChoreoTraj.java` — total times now Test 4.69 s,
  Left_Trench 10.48 s, up from 3.93 s and 9.13 s).

Net measured effect in simulation: mean tracking error 0.63 m → 0.11 m, peak 1.10 m → 0.30 m.

### Clock-governed trajectory follower

- **Added `GovernedTrajectoryCommand`** (`auton/GovernedTrajectoryCommand.java`), and
  reworked `Autos.java` to use it in place of ChoreoLib's `AutoFactory`/`AutoTrajectory`
  playback. `Drive.followChoreoSample()` gained an overload taking a governor factor.

  Even with good tuning, trajectory tracking has irreducible residual error. On the
  `Left_Trench` path a slight divergence could let a bumper corner catch a field
  obstacle; the stock time-based follower then kept advancing its setpoint, wound the
  position controller up, and commanded the drive to ~16 m/s with no recovery.

  The governed follower advances trajectory time at `dt · f(error)` instead of at
  wall-clock rate: when the robot falls behind, `f` drops toward 0 and the setpoint
  *pauses* so the robot can catch up, rather than racing away. The velocity feedforward
  is scaled by `f` (acceleration by `f²`) to match the slowed setpoint. Event markers are
  re-implemented on the governed clock (ChoreoLib ties them to its private wall-clock
  timer); a stuck-timeout ends the command gracefully if the robot is permanently caught.

  Across four simulation runs of `Left_Trench`, the previously unrecoverable runaway
  (≈3 m / 111° final error) no longer occurred — every run recovered and completed.

### Tooling and environment

- **Added `scripts/nt_capture.py` and `scripts/nt_analyze.py`** — NetworkTables capture
  and analysis tools used to diagnose trajectory tracking against the running simulator.
- **Enabled the simulation GUI** (`build.gradle`): `wpi.sim.addGui().defaultEnabled`
  false → true, so the SimGUI window opens for interactive testing. Note: with the GUI
  enabled, AdvantageKit log replay does not work — set this back to `false` to use replay.
- **`.gitignore`**: added `gradle.properties`, a local machine-specific JDK pin
  (`org.gradle.java.home` pointing at the JDK 17 install) that should not be committed.
- **`gradlew`**: set the executable bit (file mode 644 → 755) so the Gradle wrapper runs
  directly on Linux.

### Other

- **`RobotContainer.java`**: annotated the `vision` field with
  `@SuppressWarnings("unused")`. The `Vision` subsystem runs via its registered
  `periodic()`; the field itself is never read after construction, so the annotation
  silences the unused-field warning.
