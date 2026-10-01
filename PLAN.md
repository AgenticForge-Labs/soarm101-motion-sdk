# Development plan

Robo Director is the primary source of cross-repository integration requirements. The motion SDK remains authoritative for hardware, calibration, kinematics, planning, and safety; Studio and Puppeteer own the adapters that expose those abilities to Director.

## Implemented motion foundation

- [x] Direct official Feetech SDK hardware backend.
- [x] STS3215 IDs, register table, model verification, and synchronized commands.
- [x] Native and LeRobot-compatible calibration storage/discovery with content-addressed
  calibration IDs, immutable history, and motion-artifact provenance.
- [x] Five-joint arm and `SO101Gripper` tool separation.
- [x] FK, bounded IK, minimum-jerk joint movement, and preplanned linear movement.
- [x] Blocking and nonblocking motion handles with unified cancellation.
- [x] Torque-safe enable, trajectory validation, feedback completion, timeouts, diagnostics, CLI, and simulation.
- [x] PySide6 Setup/Control/Teach workspaces with shared follower/leader GUI
  mechanical-stop calibration.
- [x] Persistent Home/Rest pose library and calibrated joint-control ranges.
- [x] Independent read-only leader/controller connection and selectable teaching source.
- [x] Named taught points captured from follower or leader and replayed as joint or linear moves.
- [x] Immutable 50 Hz trajectory recording with gripper and optional effort/current diagnostics.
- [x] Validated trajectory replay with safe pre-roll to the recorded start pose.
- [x] Trajectory Editor v1: timeline, scrub, crop selection, speed scaling, Save As, and replay.
- [x] Persistent sequence editor/runner with point, Home/Rest, gripper, wait, trajectory,
  and semantic primitive steps plus step execution, repeat, speed scaling, pause/resume,
  cancellation, and STOP/HOLD integration.
- [x] Guarded 50 Hz joint streaming and leader-to-follower teleoperation in relative or
  absolute calibrated modes, with optional gripper mirroring.
- [x] Advanced non-destructive trajectory editing: smoothing, delete/splice, holds,
  keyframes, markers, repeated clips, and semantic motion primitives.
- [x] Leader FREE/PARKED state, cross-arm pose matching, no-motion relative relink, and
  coarse leader → fine Manual transfer workflow.
- [x] Persistent Manual gripper panel and one synchronized GUI gripper-speed preset across
  manual, teleoperation, saved-pose, sequence, and trajectory-replay paths.
- [x] Position-first GUI Programs workflow over the existing MotionSequence engine, with
  saved-position Move steps, Open/Close/custom gripper actions, waits, per-Move speed,
  and advanced recorded-motion steps.
- [x] Persistent right-hand live follower sidebar across every GUI tab, with modern
  styling, current joints/TCP/gripper/status, always-available follower controls, and
  contextual ghost overlays for leader/saved/recorded/program poses.
- [x] Radial shoulder-pan Program pattern generator that inherits a saved base pose and
  overrides only shoulder pan across an inspectable series of guarded Move steps.
- [x] Shared USB-camera foundation with named persisted camera profiles, one GUI worker per
  physical device, concurrent multi-camera streaming, Camera/Teleoperation named views, and
  CLI discovery/configuration/named or all-camera still capture.
- [x] Shared machine-local workstation profile for follower/leader addressing, calibration
  references, and named cameras consumed by GUI, CLI, and external agents.
- [x] Model-agnostic `agent-as-code/` machine setup and multi-view capture helper plus a
  neutral SDK-level agent CLI contract that does not prescribe task strategy.

## Director ecosystem integration 0.2

- [x] Add `.agenticforge/integration-manifest.json` and `INTEGRATION.md`.
- [x] Publish stable units, base-frame, resource, stop-classification, and TCP-ownership metadata.
- [x] Define this SDK as a library consumed by Studio and Puppeteer rather than a Director service.
- [x] Expose a shared `motion-platform:<id>` resource convention.
- [x] Document nonblocking handle and cancellation requirements for consumer adapters.
- [x] Add integration metadata tests.
- [ ] Validate the Studio SO-ARM motion-platform plugin in simulation.
- [ ] Validate the Puppeteer named-gesture embodiment in simulation.
- [ ] Define and validate the trajectory/demo-data adapter from this SDK's recorded
  leader/follower trajectories into Robo Puppeteer training and motion-data workflows.
- [ ] Pass the Director cross-repository simulated-show suite.

## CLI and GUI feature parity

The CLI and GUI must call the same SDK operations and saved libraries. The GUI keeps
its own persistent connection for live controls; it must not launch CLI subprocesses.

- [x] Match basic setup, structured readout, read-only IK, joint motion, Cartesian jogs,
  and gripper motion, including simulation-capable one-off CLI checks.
- [x] Add CLI arm discovery, absolute Cartesian moves, named pose capture/replay,
  trajectory replay, sequence execution, and effort status.
- [ ] Add CLI trajectory recording, non-destructive editing, and primitive management.
- [ ] Add CLI sequence and named pose library editing beyond capture/replay.
- [ ] Add CLI effort guard configuration, peak reset, and trip clearing.
- [x] Add CLI camera discovery, shared settings, and fresh still-frame capture.
- [ ] Add a persistent CLI session for STOP/HOLD, pause/resume, and guarded leader-to-follower teleoperation.
- [ ] Verify matching behavior in simulation and fake transport before hardware use.

## Physical validation gate

- [ ] Run port discovery and six-motor model check on the user's arm.
- [ ] Import or perform calibration and verify all directions.
- [ ] Run the conservative 2° CLI smoke test on each joint with no payload and verify
  physical direction before Cartesian motion.
- [ ] Validate stop, completion timeout, and torque-safe enable on hardware.
- [ ] Validate FK against measured TCP positions.
- [ ] Complete the paper/workspace calibration: four table corners plus a physically
  measured UP reference tied to the active motor calibration.
- [ ] Run the supervised elevated-paper Cartesian-linear traversal using known-reachable
  FK endpoints derived from the taught D→UP joint delta
  (D_UP→A_UP→B_UP→C_UP→D_UP→CENTER_UP), including replay from an arbitrary ordinary
  resting pose, and review smoothness plus the reported affine-estimated endpoint heights.
- [ ] If exact constant physical height across the full paper is required, extend teaching
  to collect additional elevated physical correspondences rather than inferring all heights
  from the single D_UP measurement.
- [ ] Resolve any remaining physical/model Cartesian-direction mismatch before broader
  low-speed linear paths/tolerance validation.
- [ ] Validate guarded leader-to-follower streaming at low speed, including STOP,
  leader-readout loss, gripper mirroring, and stream safety trips.
- [ ] Validate sequence execution and edited motion primitives on hardware.
- [ ] Validate the selected USB camera on the target workstation, confirm negotiated
  resolution/FPS/FourCC, live GUI preview in Camera and Teleoperation, and still capture.
- [ ] Tag the first hardware-validated alpha release.

## Later

- [ ] Revisit optional agent sandboxing only after direct CLI/camera robot loops are
  physically validated. The earlier OpenShell experiment is deferred and is not an
  architectural dependency for initial agent control.
- [ ] Investigate true leader gravity compensation as a separate feature. Compare the
  Trossen leader/SDK approach before selecting a control strategy; do not couple this to
  the current position-hold parking implementation.
- [ ] Parallel gripper implementation.
- [ ] Tool assemblies with camera and gripper TCPs.
- [ ] Optional collision geometry and self-collision checks.
