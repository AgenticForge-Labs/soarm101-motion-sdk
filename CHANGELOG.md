# Changelog

## Unreleased

- Strengthened live mechanical-stop calibration with per-motor sweep thresholds:
  all five pose joints now require at least 2048 encoder ticks (180°) of observed
  stop-to-stop travel; the shorter gripper keeps a separate provisional threshold.
- Added six live pie-style calibration gauges showing observed span, provisional target,
  percent coverage, and PASS state while encoder extrema are recorded.
- Hardened Feetech torque enable: every selected motor's measured Present_Position is
  checked against its active EEPROM Min/Max Position Limits before any Goal_Position,
  EEPROM lock, or Torque_Enable write. Out-of-range or invalid limits now fail closed
  with a SafetyViolationError to prevent firmware-clamped startup movement.
- Added a persistent sequence editor/runner with point, Home/Rest, gripper, wait,
  trajectory, and semantic primitive steps; supports Run Step, repeat, speed scaling,
  step-boundary pause/resume, cancellation, and STOP/HOLD.
- Added guarded 50 Hz joint streaming and leader-to-follower teleoperation with
  relative/clutch-safe and absolute calibrated mappings plus optional gripper mirroring.
- Added advanced non-destructive trajectory editing for smoothing, delete/splice, holds,
  keyframes, markers, repeated clips, and semantic motion primitives.
- Added named taught points captured from follower or leader, with joint or Cartesian replay.
- Added immutable 50 Hz trajectory recording, optional motor effort/current diagnostic channels,
  validated exact replay, and non-destructive raw/edited trajectory libraries.
- Added a PySide6 trajectory timeline/editor with scrub, crop selection, speed scaling,
  Save As, and replay of full or selected clips.
- Added Setup/Control/Teach GUI workspaces, persistent Home/Rest poses, calibrated joint-slider ranges, and independent leader-arm readout.
- Unified GUI and CLI calibration on the mechanical-extrema midpoint method with encoder seam unwrapping.
- Added TESTING.md as the staged handoff checklist for deferred physical validation.

- Added Agentic Forge Director 0.2 integration metadata, library manifest, resource identity, frame convention, stop classification, and TCP ownership rules.
- Documented the Studio and Puppeteer adapter responsibilities while keeping the SDK independent of Director.
- Replaced the LeRobot runtime plan with direct official Feetech SDK control.
- Added calibration compatibility, stock-gripper tooling, FK, bounded IK, smooth joint and linear motion, diagnostics, CLI workflows, and simulation.
- Hardened torque enable by latching measured positions before energizing the servos.
- Unified blocking and nonblocking motion cancellation and made software stop wait for the active motion task to terminate.
- Added complete command-rate trajectory prevalidation, joint speed/acceleration checks, and feedback-based completion timeouts.
- Fixed anti-parallel camera and approach-axis IK constraints.
- Made calibration transactional with EEPROM rollback on failure or interruption.
- Serialized Feetech transport access and made torque transitions transactional.
- Added in-motion fault, following-error, direction, and command-deadline monitoring.
- Added absolute motion safety ceilings and independent compatible-orientation residuals.
- Made stock-gripper operations cancellable through the arm-level stop lifecycle.
- Added one-time direct motor ID/baud setup and explicit motor configuration commands.
- Made ordinary connections and diagnostics configuration-neutral by default.
- Pinned the reviewed Feetech transport package to version 2.0.0.
- Added conservative floor/base/reach/self-clearance checks and a physical FK validation recorder.
- Replaced the misleading `emergency_stop` alias with `software_stop`.
- Added a guided one-joint hardware smoke-test command.
- Added `soarm101-setup`, a single guided diagnose/configure/calibrate workflow for assembled arms.
- Made isolated-motor setup fast by default and added best-effort EEPROM relocking after failures.
- Separated ordinary torque control from EEPROM locking so relax, disconnect, and rollback never leave persistent motor settings unlocked.
- Added an optional PySide6 controller with joint sliders, world/tool Cartesian linear jogs, absolute world poses, responsive stop, simulation, and gripper controls.
- Added matching `soarm101 jog` and `soarm101 gripper` CLI actions using shared frame-transform semantics.
- Reused the workspace-validated Cartesian plan for execution so GUI and CLI linear moves run IK and time-parameterization only once.
