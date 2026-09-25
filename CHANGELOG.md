# Changelog

## Unreleased

- Reworked Cartesian jogging around small, repeatable bench motion: the GUI now defaults
  to 2 mm translation steps, accepts up to 32 queued jog clicks while the current jog
  completes, executes them sequentially through the existing guarded motion path, and
  clears the queue on STOP/HOLD, relax, disconnect, or jog failure.
- Added requested-versus-achieved TCP diagnostics and an interactive 3D kinematic view to
  the Cartesian tab. The view uses the same native SO-101 FK model as planning and shows
  the current target separately from measured state.
- Added full-path 2 mm world-axis regression tests so +X/-X, +Y/-Y, and +Z/-Z must produce
  distinct requested Cartesian displacement in simulation.
- Added a configurable 0.5 mm Cartesian IK position tolerance for planned linear paths.

- Added content-addressed calibration provenance: every physical calibration has a
  SHA-256 ID, the current calibration file is retained for compatibility, and immutable
  history snapshots are archived per robot ID.
- Bound saved Home/Rest poses, taught points, raw/edited trajectories, primitives, and
  sequences to their source/target calibration context. Physical replay now fails closed
  for legacy, stale, or cross-robot-unbound artifacts; simulation remains permissive.
- Moved trajectory and sequence provenance checks into the public replay APIs so scripts
  and higher-level consumers cannot bypass the GUI safety gate.
- Routed Home/Rest through guarded saved-pose replay and sequence the stored gripper
  command only after the arm reaches the saved joint pose.
- Added a backend torque-enable refusal for motors that still use factory 0..4095
  calibration ranges, even when connected through the setup/uncalibrated path.
- Added a first-physical-run bench card and tightened TESTING.md around ordered stop
  gates, 2° joint smoke tests, small initial gripper motion, and calibration-ID checks.
- Unified follower and leader mechanical-stop calibration in the Setup tab. A single
  target selector routes the same sweep algorithm, live gauges, and pass criteria to
  either arm while preserving separate robot IDs and calibration files.
- Added a leader setup-only "allow uncalibrated connection" option so a fresh leader can
  connect for calibration without weakening its normal read-only teaching workflow.
- Added a Setup GUI motor-effort safety/characterization panel with per-motor live
  current/load, session peaks, effective thresholds, latched-trip visibility, manual
  refresh/reset/clear actions, and explicit session-only global threshold controls.
- Effort threshold changes now require torque OFF; changing settings never clears a
  latched trip, and disabling the effort guard requires an explicit GUI confirmation.
- Cached effort readings/peaks are updated by the existing guarded motion monitor so the
  GUI can display characterization data without adding a competing serial-read loop.
- Fixed LeRobot calibration coordinate compatibility: arm-joint zero is now the
  midpoint of each calibrated `range_min` / `range_max` pair rather than always
  encoder tick 2047. Asymmetric imported LeRobot calibrations therefore preserve
  their recorded midpoint as 0 rad, while native symmetric mechanical-stop
  calibrations remain effectively unchanged.
- Made leader→follower streaming rate explicit: the software now defaults to 20 Hz with
  5/10/20/50 Hz GUI choices. First hardware validation still proceeds 5→10→20 Hz, and
  50 Hz remains experimental with an explicit confirmation.
- Stream speed/acceleration checks now use the selected teleop rate rather than the
  separate 50 Hz planned-trajectory clock.
- Added stale-sample and repeated-cycle-overrun guards so serial backlog terminates
  teleoperation and holds the follower instead of executing increasingly delayed commands.
- Added live teleop cycle-time/sample-age reporting and a documented 5→10→20→50 Hz
  hardware validation/optimization ladder.
- Strengthened live mechanical-stop calibration: all five pose joints require at least
  2048 encoder ticks (180°), the gripper requires 900 ticks, and every actuator must
  complete two full end-to-end traversals before a calibration can be saved.
- Updated the six live calibration gauges to show two-traversal progress, use realistic
  display spans (including the larger wrist-roll travel), and reuse the selected arm's
  prior gripper range for display scaling without changing pass/fail thresholds.
- Calibration now defaults to a 90-second time limit, ends early when all six actuators
  reach 2/2, and aborts on servo input-voltage faults while preserving the previous
  calibration on cancel or failure.
- Hardened Feetech torque enable: every selected motor's measured Present_Position is
  checked against its active EEPROM Min/Max Position Limits before any Goal_Position,
  EEPROM lock, or Torque_Enable write. Out-of-range or invalid limits now fail closed
  with a SafetyViolationError to prevent firmware-clamped startup movement.
- Added a persistent sequence editor/runner with point, Home/Rest, gripper, wait,
  trajectory, and semantic primitive steps; supports Run Step, repeat, speed scaling,
  step-boundary pause/resume, cancellation, and STOP/HOLD.
- Added guarded leader-to-follower joint streaming with 5/10/20/50 Hz selection,
  relative/clutch-safe and absolute calibrated mappings, guarded startup alignment,
  host-side target smoothing, and optional contact-aware gripper mirroring.
- Added advanced non-destructive trajectory editing for smoothing, delete/splice, holds,
  keyframes, markers, repeated clips, and semantic motion primitives.
- Added named taught points captured from follower or leader, with joint or Cartesian replay.
- Added immutable 50 Hz trajectory recording, optional motor effort/current diagnostic channels,
  validated exact replay, and non-destructive raw/edited trajectory libraries.
- Added a PySide6 trajectory timeline/editor with scrub, crop selection, speed scaling,
  Save As, and replay of full or selected clips.
- Evolved the GUI into Setup, Manual, Teleoperation, Record / Teach, Edit recordings, Run, and Log workspaces, with persistent Home/Rest poses, calibrated joint-slider ranges, independent leader-arm readout, and persistent session logging.
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
- Expanded CLI/GUI parity with arm discovery, absolute Cartesian moves, named pose capture/replay, trajectory replay, sequence execution, and effort-status inspection through shared SDK operations.
- Added read-only TCP/table repeated-point calibration support in `examples/tcp_table_calibration.py` for separating base/table offsets from pose-dependent TCP or kinematic error.
- Reused the workspace-validated Cartesian plan for execution so GUI and CLI linear moves run IK and time-parameterization only once.
