# Agent instructions

This repository is used in two different ways: agents may **develop the SDK**, or they may
**operate and inspect an SO-ARM101 through the SDK/CLI**. Keep those roles separate. Code
changes require development discipline; robot use requires conservative hardware behavior.

## Before doing anything

- Read `README.md` for the current user workflow.
- Read `docs/safety.md` before any real-arm motion.
- Read `docs/physical-run.md` before first hardware validation or after a meaningful
  calibration/hardware change.
- Read `PLAN.md` and `docs/architecture.md` before changing architecture or adding features.
- Do not assume a command or capability exists. Check the current CLI help, SDK API, or source.

## Development

1. Keep ROS, camera capture, tracking, OBS, and show orchestration out of this repository.
2. Keep LeRobot optional and reference-only; do not import it from runtime code.
3. Treat the arm as five pose joints plus tool actuators.
4. Never add a motion method that reports success without executing documented behavior.
5. Validate complete Cartesian paths before motor commands.
6. Add simulator tests and fake-transport tests before physical hardware tests.
7. Preserve calibration and third-party attribution compatibility.
8. Do not claim physical validation until a real-arm smoke test passes.
9. Treat calibration IDs as motion provenance; physical replay must fail closed on missing
   or mismatched provenance.
10. Keep GUI and CLI features on shared SDK operations and saved libraries. The GUI may own
    persistent hardware sessions, but it must not shell out to CLI subprocesses for robot
    behavior.
11. Preserve the distinction between planned motion and live-streaming safety. Do not weaken
    joint, step, rate, acceleration, following-error, fault, effort, calibration, or
    provenance checks merely to make a new workflow pass.
12. When user-visible behavior, UI names, defaults, calibration rules, CLI behavior, or safety
    gates change, update `README.md`, `CHANGELOG.md`, `TESTING.md`, and the relevant
    files under `docs/` in the same change.
13. Keep tests and lint green on the supported Python versions before considering a change
    complete.

## Using the CLI / operating the arm

1. Prefer inspection before motion. Start with commands such as `soarm101 ports`,
   `soarm101 diagnose`, and read-only state inspection before enabling torque.
2. For a new or changed arm, use the documented setup/calibration workflow rather than
   bypassing calibration checks. A valid calibration and matching calibration provenance
   are prerequisites for physical replay.
3. For the first powered motion, use `soarm101 smoke-test` one joint at a time, with no
   payload, a clear workspace, and physical power immediately reachable.
4. Do not open the same serial port from the GUI and CLI at the same time.
5. Use conservative speed, acceleration, motion size, and streaming rates until that exact
   hardware setup has been validated. Software STOP/HOLD is not an emergency stop.
6. Never disable or work around calibration, joint-limit, following-error, fault, effort,
   provenance, or other safety checks simply to make a command run.
7. Saved Home/Rest poses, taught points, trajectories, primitives, and sequences are tied to
   calibration context. If replay is rejected after recalibration, review and deliberately
   recreate/rebind the artifact instead of bypassing the guard.
8. Treat simulation as software validation only. A successful simulated command is not
   evidence that the same motion is physically safe.
9. Use only CLI capabilities that currently exist. If an operation is GUI-only or not yet
   implemented in the CLI, do not invent a command or emulate it by bypassing the SDK.
10. When scripting with the Python API or CLI, reuse the SDK's guarded operations rather
    than writing directly to servo registers unless the task is explicitly low-level
    hardware development and the safety implications are understood.

## Repository boundary

This SDK owns SO-ARM101 motion, calibration, kinematics, tooling/TCP definitions,
diagnostics, teaching/replay primitives, and their safety/provenance rules. Higher-level
camera systems, perception, show control, and cross-robot orchestration belong in their
respective AgenticForge repositories.
