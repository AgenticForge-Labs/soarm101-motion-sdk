# Safety

This is experimental software for a low-cost hobby/educational robot arm, not a certified industrial controller.

- Clear the workspace and remove payloads during initial tests.
- Start near the centered pose with small commands and low speeds.
- Keep physical power accessible.
- Do not depend on software stop as an emergency stop.
- Do not force joints during calibration.
- Confirm motor voltage, calibration, direction, TCP, and limits on the exact assembly.
- Inspect voltage, temperature, current, and status using `soarm101 diagnose`.

## Runtime safeguards

- Normal connection and read-only diagnosis do not rewrite motor configuration.
- Direct hardware writes are rejected while torque is disabled.
- Enabling torque latches measured positions as goals. If a position is no more than
  8 encoder ticks (about 0.7°) outside a calibrated EEPROM limit, its goal is clamped
  inward to that limit and the GUI logs the correction. Larger violations still block
  enable before any goal or torque write.
- Feetech transport and synchronized writes use one reentrant lock.
- Joint and Cartesian trajectories are preplanned and checked before motion.
- Overrides cannot exceed absolute host-side speed/acceleration ceilings.
- Active trajectories monitor faults, following error, unexpected direction, and deadline overruns.
- Motion failures issue a best-effort hold.
- `wait=True` verifies measured completion.
- Stock-gripper moves participate in the arm-level stop lifecycle.
- Calibration snapshots and restores motor EEPROM on failure when possible.
- Torque enable rolls back motors already energized when a later enable fails.

## Calibration provenance

Each physical calibration has a deterministic SHA-256 identity derived from motor ID,
drive mode, homing offset, and calibrated encoder limits. Successful calibration saves
the normal current file and an immutable fingerprinted history copy.

Persistent motion artifacts record the calibration context in which they were created.
Same-arm artifacts must match the connected robot's active calibration. Leader-recorded
artifacts additionally carry the follower calibration they were approved to target.
After recalibration, stale physical replay is rejected until the artifact is explicitly
reviewed and saved/bound again.

Legacy artifacts without calibration provenance are intentionally blocked on physical
hardware. Simulation skips this gate so old software fixtures and editing workflows remain
usable.

A setup connection that still uses factory `0..4095` ranges is also torque-inhibited at
the backend. The setup/uncalibrated option allows readout and calibration only; it is not
a way to bypass powered-motion calibration requirements.

## Motor effort guard

The managed Feetech backend monitors raw STS3215 `Present_Current` and signed
`Present_Load` while torque is enabled. These are useful contact/collision indicators,
but they are **not calibrated force or torque measurements**.

The Setup GUI exposes a characterization panel with:

- current and signed load for each of the six motors;
- session peak current and peak absolute load;
- each motor's effective current/load trip thresholds;
- the latched trip reason;
- explicit Refresh readings, Reset peaks, and Clear latched trip actions; and
- session-only global guard settings.

Threshold changes require torque OFF. Disabling the effort guard requires explicit GUI
confirmation and affects only the current/load interlock; joint/calibration limits,
workspace checks, following-error monitoring, motor faults, command timing, and other
motion protections remain active.

A threshold change never clears a latched trip. Remove the obstruction first, then use
**Clear latched trip** explicitly. Clearing the latch does not itself command motion.

The current default thresholds are starting guardrails rather than validated physical
limits. Characterize the exact arm at low speed/no payload before interpreting them as
appropriate operating values. See `TESTING.md`.

## Coarse geometry envelope

The SDK checks every requested joint path against a conservative centerline model:

- elbow, wrist, and TCP stay above a configured floor plane;
- the TCP stays inside a maximum radial reach;
- distal points stay outside a base keep-out cylinder;
- nonadjacent link centerlines maintain a minimum clearance.

These checks catch gross fold-back and table/base hazards. They are not mesh-level collision detection and cannot model printed-part variation, cables, payloads, external fixtures, backlash, or compliance. They remain opt-in for live joint streaming through `teleop_workspace_checks` until the robot-specific table frame and tool geometry are calibrated. Other joint, step, rate, acceleration, following-error, fault, and effort checks remain active.

The API uses `stop()` and `software_stop()`. It intentionally does not expose `emergency_stop()` because a Python command cannot replace a physical power or enable circuit.
