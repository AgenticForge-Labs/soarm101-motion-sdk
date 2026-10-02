# Safety

This is experimental software for a low-cost hobby/educational robot arm, not a certified industrial controller.

- Clear the workspace and remove payloads during initial tests.
- Start near the centered pose with small commands and low speeds.
- Keep physical power accessible.
- Do not depend on software stop as an emergency stop.
- Do not force joints during calibration.
- Confirm motor voltage, calibration, direction, TCP, and limits on the exact assembly.
- Executable pose-joint authority uses the nominal model/calibration range. Extra
  calibration-derived travel is permitted only for explicitly validated joints; the
  current default allowlist is `wrist_flex`, which retains a 4° margin inside its measured
  mechanical stop. Other joints stay at the normal model/calibration intersection until
  separately characterized. Never treat a measured mechanical stop as a command target.
- Inspect voltage, temperature, current, and status using `soarm101 diagnose`.

## Transient torque-control replies

The Feetech transport may occasionally lose or corrupt the status reply to an idempotent
`Torque_Enable` or `Lock` write even when the servo received the requested value. For these
two control bits only, the backend performs bounded recovery: read back the register after a
communication error; if the requested value is already present, continue; otherwise retry the
same write once. Persistent or unverifiable communication failure still aborts torque enable
and triggers rollback. Motion targets and other register writes do not inherit this retry.

## Leader parking and handoff

The leader normally connects with torque off so it can be moved by hand. **Park leader**
is a powered operation: the SDK first latches the measured position as the servo goal and
then enables torque so the leader holds that pose. **Release leader** turns torque off.

Cross-arm pose matching is also powered motion. The source pose is read fresh when the
button is pressed, but the destination arm still applies its own calibration, joint,
workspace, speed, acceleration, fault, effort, and completion guards. Matching one arm to
the other is not collision-aware with respect to the second physical arm, cameras, tables,
cables, payloads, or other workspace objects.

Use **Relink here — no motion** when the two arms are intentionally at different poses and
you want relative teleoperation without moving either into alignment. The follower still
enables torque by latching its own current measured pose before streaming begins. Starting
live teleoperation releases a parked leader so it is back-drivable again.

## Servo hold authority

The recommended STS3215 position P coefficient is 32, matching the motor factory default.
A lower P gain can leave a gravity/static-friction deadband where a joint approaches a
target but cannot settle within the SDK tolerance. Do not compensate for visible sag or
several-degree final error merely by extending motion timeouts.

The explicit `soarm101 configure` command applies recommended servo settings with torque
disabled; normal connections remain configuration-neutral.

For supervised paper Cartesian validation, a motion exception transitions to STOP/HOLD
and waits for the operator before relaxing, so a failed gravity-loaded move does not
immediately drop the arm.

## Runtime safeguards

- Normal connection and read-only diagnosis do not rewrite motor configuration.
- Direct hardware writes are rejected while torque is disabled.
- Enabling torque latches measured positions as goals. A relaxed mechanism can settle a
  few encoder counts beyond an EEPROM endpoint because of quantization/backlash. If a
  measured position is no more than 8 encoder ticks (about 0.7°) outside a calibrated
  EEPROM limit, only the startup latch goal is clamped inward to that limit. Larger
  violations still block enable before any goal or torque write, and ordinary commanded
  positions remain constrained to the calibrated limits.
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

These checks catch gross fold-back and table/base hazards. They are not mesh-level collision detection and cannot model printed-part variation, cables, payloads, external fixtures, backlash, compliance, or a wrong physical/model Cartesian mapping.

The default coarse floor is a **model-space assumption**, not a measured physical table.
Do not lower it merely because a manually touched table point reports negative model Z,
and do not assume model +Z is physical up. Hardware testing produced a case where a
numerically valid +Z hover physically moved laterally into the table. Use the manual
paper/workspace calibration to measure the table and a physical-UP reference first; a
new workspace calibration remains unvalidated for autonomous Cartesian motion until a
separate supervised direction test passes.

Live-stream workspace checks remain opt-in through `teleop_workspace_checks` until the
robot-specific table frame and tool geometry are calibrated. Other joint, step, rate,
acceleration, following-error, fault, and effort checks remain active.

The API uses `stop()` and `software_stop()`. It intentionally does not expose `emergency_stop()` because a Python command cannot replace a physical power or enable circuit.
