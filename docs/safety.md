# Safety

This is experimental software for a low-cost educational robot arm, not a certified industrial controller.

- Clear the workspace and remove payloads during initial tests.
- Start near the centered pose with small commands and low speeds.
- Keep physical power accessible.
- Do not depend on software stop as a certified emergency stop.
- Do not force joints during calibration.
- Confirm calibration, direction, TCP, and joint limits on the exact printed assembly.
- Inspect voltage, temperature, current, and status using `soarm101 diagnose`.

## Runtime safeguards

- Direct hardware writes are rejected while torque is disabled.
- Enabling torque first writes each servo's measured position back as its goal, preventing stale-goal jumps.
- Feetech packet traffic and synchronized writes are serialized through one reentrant transport lock.
- Joint and Cartesian trajectories are sampled and checked for limits, command steps, velocity, and acceleration before the first motion packet.
- Per-command speed and acceleration overrides cannot exceed configured absolute safety ceilings.
- Active trajectories monitor motor faults, following error, unexpected direction, and command deadline overruns.
- Blocking and nonblocking moves share one tracked cancellation token; `stop()` waits for the motion task to terminate.
- Motion failures issue a best-effort hold before returning the original exception.
- `wait=True` verifies measured joint positions and motor moving state before reporting completion.
- Stock-gripper moves are cancellable and are included in the arm-level stop lifecycle.
- Calibration snapshots motor EEPROM and restores it if calibration fails or is interrupted.
- Torque enable rolls back motors already energized if a later motor fails to enable; torque disable attempts every selected motor before reporting errors.

These safeguards reduce software risk but do not replace a physical emergency stop, current-limited power, mechanical inspection, or supervised first-arm testing.
