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

1. Keep ROS, perception/tracking, OBS, and show orchestration out of this repository. Basic USB camera discovery, configuration, live preview, and still-frame capture are first-class SDK capabilities shared by CLI and GUI.
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
    persistent hardware and named camera sessions, but it must not shell out to CLI subprocesses
    for robot or camera behavior. Workstation addressing/camera settings have one persisted
    source of truth; each physical camera has at most one GUI worker and views reuse that session.
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
11. Do not infer physical up or table height from model/base +Z. The supervised
    elevated-paper validation uses the manually measured D_UP point to calibrate physical
    workspace Z and software-levels A/B/C/D/center to that same workspace height while
    preserving calibrated workspace X/Y. Read-only IK endpoint preflight must accept every
    corrected target before powered replay. Replay performs one separately preflighted
    calibrated-workspace-Z startup clearance before lateral travel, then validates the
    actual Cartesian primitive with A_UP -> B_UP -> C_UP -> D_UP -> CENTER_UP through
    `move_linear()` and position-only IK. Do not replace those paper segments with joint
    replay merely to avoid Cartesian behavior: the experiment exists to validate Cartesian
    linear motion. The paper workflow uses the known-smooth 20 Hz host cadence. Planned
    Cartesian execution must use the same responsive servo-side tracking profile as live
    teleoperation (STS3215 speed_raw=0, acceleration_raw=254); host trajectory planning owns
    speed and acceleration shaping. Hardware did not show a benefit from the temporary
    bounded-acceleration launch in #73, so Cartesian timing uses the symmetric half-cosine
    profile again. Hardware after #74 remained very shaky, but review of the public
    managed controller confirmed that real `move_linear()` execution was already using
    the documented teleoperation profile (speed_raw=0, acceleration_raw=254). Treat the
    shake as unresolved rather than changing servo behavior by assumption. If forward
    position-only IK hits a numerical pocket while the endpoint is known reachable,
    deterministic planning may use the exact
    read-only endpoint solution as a reverse boundary condition, but it must re-solve every
    Cartesian sample at the unchanged hard tolerance and reconnect continuously to the
    measured start. Hardware #75 showed the forward and reverse solves converging to nearly
    the same ~0.65 mm residual, so endpoint reachability alone is not evidence that the
    intervening straight line is feasible. Before any further powered elevated replay,
    preflight every full paper segment read-only and inspect the failing sample's XYZ
    residual, Jacobian conditioning, and effective joint-limit margin. Position-only
    smoothness work belongs in Cartesian-constrained IK
    reprojection/diagnostics, not motor PID or another unmeasured launch heuristic. The calibrated workspace is authoritative for paper
    height; the generic model workspace may remain destination-only where documented. All
    joint, IK continuity, step/rate/acceleration, following-error, effort, fault,
    communication, settle, provenance, and timing guards remain active. Broader autonomous
    Cartesian use remains unvalidated until the hardware evidence is reviewed.
12. Keep `docs/agent-arm101-cli.md` and `cli_use.md` technical and policy-neutral. They
    document the CLI contract, common commands, units, coordinate semantics, outputs, and
    enforced safety behavior. Task-solving strategy and benchmark policy remain outside
    those neutral references. Robot-specific agent runtime/security setup belongs in
    `docs/agent-sandbox.md`; higher-level benchmark design and scoring remain outside the SDK.
13. The canonical OpenShell path must never mount the Motion SDK checkout, serial devices,
    camera devices, calibration files, Docker socket, SSH credentials, or unrelated host
    files into the reasoning sandbox. The only physical action path is
    OpenShell -> robotctl -> authenticated broker -> bounded agent CLI -> SDK.
14. Human `agent arm` authority remains outside the sandbox. Sandbox setup/run code may
    verify authority but must never create, extend, or bypass it.
15. Keep agent-harness differences behind `agent_adapters.py`. Adding another agent may
    add an image recipe, provider profile, skill, mutable-home setup, and command builder,
    but must not add a second robot API, second broker, or agent-specific motion semantics.
    Hermes and Codex must share the same `robotctl -> broker -> bounded agent CLI -> SDK`
    physical path.

## Repository boundary

This SDK owns SO-ARM101 motion, calibration, kinematics, tooling/TCP definitions,
diagnostics, teaching/replay primitives, their safety/provenance rules, basic local USB
camera capture used to observe the arm workspace, and the canonical constrained agent
execution boundary for this device (bounded CLI, broker/client, and robot-specific OpenShell
environment). Higher-level perception/tracking, calibrated
multi-camera stage systems, show control, and cross-robot orchestration belong in their
respective AgenticForge repositories.
