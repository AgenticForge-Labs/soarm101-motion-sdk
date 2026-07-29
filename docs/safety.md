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
- Enabling torque first writes each measured servo position back as its goal.
- Feetech transport and synchronized writes use one reentrant lock.
- Joint and Cartesian trajectories are preplanned and checked before motion.
- Overrides cannot exceed absolute host-side speed/acceleration ceilings.
- Active trajectories monitor faults, following error, unexpected direction, and deadline overruns.
- Motion failures issue a best-effort hold.
- `wait=True` verifies measured completion.
- Stock-gripper moves participate in the arm-level stop lifecycle.
- Calibration snapshots and restores motor EEPROM on failure when possible.
- Torque enable rolls back motors already energized when a later enable fails.

## Coarse geometry envelope

The SDK checks every requested joint path against a conservative centerline model:

- elbow, wrist, and TCP stay above a configured floor plane;
- the TCP stays inside a maximum radial reach;
- distal points stay outside a base keep-out cylinder;
- nonadjacent link centerlines maintain a minimum clearance.

These checks catch gross fold-back and table/base hazards. They are not mesh-level collision detection and cannot model printed-part variation, cables, payloads, external fixtures, backlash, or compliance. They can be disabled for model development with `enable_workspace_checks=False`, but should remain enabled for normal hardware use.

The API uses `stop()` and `software_stop()`. It intentionally does not expose `emergency_stop()` because a Python command cannot replace a physical power or enable circuit.
