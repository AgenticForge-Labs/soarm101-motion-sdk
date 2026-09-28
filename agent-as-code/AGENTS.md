# Agent-as-code operating instructions

This directory is for **operating an already configured SO-ARM101 through its public
CLI**, not for modifying SDK source during a benchmark run.

Before physical motion:

1. Read `CLI.md`.
2. Read `../docs/safety.md` and `../docs/physical-run.md`.
3. Read `setup.local.json` and use only the configured robot and cameras.
4. Confirm the workspace is clear and physical power can be removed immediately.
5. Begin with an observation and current robot state before commanding motion.

## Task behavior

- Accept the natural-language task as the goal. Do not add an unstated goal.
- Use any available VLM/vision capability to inspect fresh camera observations.
- Treat the camera marked `primary` as the canonical task view. Auxiliary cameras are
  additional evidence, not separate authorities.
- Observe again after meaningful physical changes instead of assuming an action worked.
- Prefer small, deliberate Cartesian jogs while establishing the relationship between
  image space and robot motion.
- Use only documented `soarm101` CLI commands for robot and camera interaction.
- Never access servos, serial registers, or USB camera backends directly to bypass the SDK.
- Never disable calibration, provenance, joint, workspace, following-error, fault, effort,
  or other safety checks to make a task succeed.
- A rejected command is evidence to re-plan, not permission to bypass a guard.
- Software STOP/HOLD is not a certified emergency stop.
- Stop issuing motion once the requested physical task is complete or safe progress is no
  longer possible.

## Camera ownership

The experiment may use one primary camera and any number of auxiliary USB cameras.
`capture_observation.py` opens them sequentially through the public camera CLI and then
closes them. Do not run the desktop GUI camera session at the same time if it owns the
same USB device.

## Benchmark integrity

Do not edit `setup.local.json`, this file, `CLI.md`, SDK source, calibration files, or
task definitions during a scored run unless the experiment explicitly makes such edits
part of the treatment. Record failures rather than silently repairing the benchmark.
