# Development plan

The motion SDK remains authoritative for hardware, calibration, kinematics, planning,
and safety. Forge Puppeteer owns higher-level physical performer/stage coordination;
historical Director/Studio integration references below describe older adapter work.

## Model, coordinates, and shaking audit — 2026-10-07

Reviewed main at `8830d9064bc6551674d1a7ec00c078a07ae73d12` and open motion
PR #79 at `583e4d554d7d7e898d83687cfb003af0ff6a7abd`. This visual correction is
independent of that PR; its motion changes must not be treated as merged behavior.

### Confirmed model and presentation findings

The planner and desktop GUI use native `SO101KinematicModel`, not a runtime URDF
parser. Optional PyBullet uses the bundled simplified URDF. Comparing 1,000 random
configurations against TheRobotStudio's official
`Simulation/SO101/so101_new_calib.urdf` gave maximum TCP position difference
0.00394 mm and orientation difference 0.000926 degrees. The reference file's last
modifying commit was `385e8d7c68e24945df6c60d9bd68837a4b7411ae`.
Nominal shoulder-lift→elbow, elbow→wrist-flex, and wrist-flex→wrist-roll origin
distances are 116.0, 135.0, and 63.7 mm. These are joint-origin distances, not housing
lengths. LeRobot's published new-calibration model contains the same chain/TCP
transforms. No evidence supports changing those link dimensions by guesswork.

- [x] Replace symmetric sliding-finger GUI geometry with a nominal fixed finger and
  rotating jaw using the official pivot/travel and mesh extents. This outline is not
  measured jaw angle or collision geometry.
- [x] Preserve scale across pose/target/ghost updates; provide explicit Fit, optional
  Auto fit, side/front/top/isometric views, and a screen-plane ruler.
- [x] Draw the active session TCP and compute numerical FK from the same joint snapshot
  as the drawing. Label the grid as model Z=0, not a measured table.
- [x] Add native/bundled-URDF parity, fixed-pivot, stable-scale, and active-TCP regression
  coverage. These checks establish software consistency, not physical accuracy.
- [ ] Add optional official CAD meshes with a switchable joint-center overlay. Preserve
  shared FK/tool authority, source revision/license, bounded rendering cost, and small
  window readability. Current schematic/primitive PyBullet visuals do not represent
  full printed housings or certified collision geometry.
- [ ] Display measured table/workspace overlays only with matching workspace and motor
  calibration provenance; do not label model Z=0 as physical ground.

### Physical coordinate investigation

Prior workstation reports include physical-to-model scales X≈0.829, Y≈0.927,
UP≈0.729; physical +50 mm D→UP produced model displacement approximately
(+11.5,+5.2,+30.9) mm. A model +Z hover also moved laterally and contacted the table.
These are reported hardware observations, not reproduced by this software audit.
Unequal scales cannot be corrected by rigid base-frame rotation/translation alone.
The cause remains unidentified.

- [ ] Collect read-only, torque-off joint/TCP correspondences at multiple heights,
  extensions, and wrist orientations using a repeatable physical base reference and
  one identified contact point. Repeat approaches from both directions to characterize
  backlash/compliance. Record raw/calibrated joints, calibration ID, active TCP,
  measurement uncertainty, payload, and model revision.
- [ ] Fit base alignment and TCP offset first; then assess joint-zero offsets, stop
  capture coverage, assembly indexing/variant, and independently measured link lengths.
  Encoder midpoint/travel calibration is not geometric calibration. Keep any geometric
  corrections separate from motor EEPROM calibration and preserve saved-motion/model
  provenance if a fitted model is adopted.
- [ ] Validate on held-out poses and multiple independent UP references. A four-corner
  plus one-UP affine fit has no independent vertical validation and may compensate only
  locally; do not promote its scale/shear to a global geometry repair.
- [ ] After measurement review, design supervised small-displacement physical-direction
  and straightness checks with before/after measurement. Preserve full-path preflight
  and physical guards. Lowering the floor, loosening IK tolerance, or substituting
  joint replay does not repair geometry.

### Linear motion and shaking investigation

The current Cartesian controller already samples the Cartesian line at command rate,
uses sequential IK with continuity checks and Cartesian-constrained smoothing, and
executes with responsive servo tracking `0/254`. No swapped-axis or mm/m conversion
error was identified in shared jogging. Software agreement does not prove physical
TCP straightness.

- [ ] Complete PR #79's outstanding supervised gates on its exact head before adopting
  its motion changes. The branch contains duplicate encoder-target coalescing,
  responsive ordinary joint tracking, less-folded default Sleep, and wrist-aware teleop.
  Prior reported final-target execution still rocked: repeated writes are not the sole
  cause. The taught sleep2 posture and later 80 deg/s, 500 deg/s² responsive-profile
  tests were reported substantially smoother; low-speed Cartesian shake remains
  unresolved. Finish existing branch validation rather than duplicating its changes here.
- [ ] Compare planned joint derivatives/reversals/quantized goals with measured joint
  position, following error, current/load, actual servo profile, and cycle timing on
  matched paths, postures, payloads, and cadences. Distinguish oscillating commands
  from oscillating mechanics under smooth commands. Compare taught teleop replay with
  Cartesian plans using the existing PR #79 diagnostic tools.
- [ ] Inspect wrist mounts/horns, fasteners, cable forces, backlash, gravity loading,
  and near-limit/folded postures. Position-only IK permits posture changes; report
  Jacobian conditioning and limit margins rather than assuming all shaking is IK.
- [ ] Consider servo tuning/path shaping only after traces identify a cause. Faster
  motion provides evidence about low-speed behavior, not a universal repair. Preserve
  joint, following-error, fault, effort, timing, workspace, and stop protections.


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
- [x] Persistent Home/Rest pose library, calibration-derived natural Sleep posture, and
  calibrated joint-control ranges.
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
  neutral SDK-level agent CLI contract.
- [x] Bounded `soarm101 agent ...` facade with human-interactive time-limited motion
  authority, `agent_*` saved poses, overhead/wrist capture, safe gripper endpoints,
  Sleep/STOP-HOLD, and measured-height-limited translational jogs.

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

- [ ] Complete physical characterization of the explicit 100/1000 host motion envelope.
  The previously host-rejected **80 deg/s, 500 deg/s²** Sleep-vs-`sleep_up` comparison
  completed successfully on 2026-10-06 with responsive 0/254 tracking and was judged much
  smoother by the operator. The first **100/1000** attempt never moved because SI conversion
  differed from the exact configured ceiling by a few floating-point ULPs; that software
  boundary bug is now fixed. Re-run 100/1000 under the unchanged following-error,
  effort/fault, calibrated-limit, workspace, STOP/HOLD, and settle safeguards before
  marking the full characterization gate complete.

- [ ] Run port discovery and six-motor model check on the user's arm.
- [ ] Import or perform calibration and verify all directions.
- [ ] Run the conservative 2° CLI smoke test on each joint with no payload and verify
  physical direction before Cartesian motion.
- [ ] Validate stop, completion timeout, and torque-safe enable on hardware.
- [ ] Validate FK against measured TCP positions.
- [ ] Complete the paper/workspace calibration: four table corners plus a physically
  measured UP reference tied to the active motor calibration.
- [ ] Run the supervised paper traversal using the existing A/B/C/D + D_UP calibration:
  one calibrated-Z startup clearance move, then constant-height Cartesian
  `move_linear()` replay A_UP→B_UP→C_UP→D_UP→CENTER_UP at 20 Hz. Verify all targets
  share the trained workspace Z and compare physical smoothness directly with teleoperation.
- [ ] Diagnose the A_UP->B_UP straight-line feasibility before any further powered
  paper replay. Hardware #75 evidence shows forward and endpoint-seeded reverse IK both
  converge near the same residual (0.648/0.646 mm) against the unchanged 0.5 mm tolerance.
  Require read-only full-segment preflight of every elevated paper segment and inspect the
  failing sample's residual vector, Jacobian conditioning, and joint-limit margin to decide
  whether this is optimizer behavior or a real model-workspace boundary. Use the torque-off
  calibration-limit comparison first: the active mechanical-stop calibration is wider than
  the nominal model on every pose joint, including wrist flex ±103.9° versus ±95°. The
  torque-off margin search has now bounded the original 107 mm path's wrist requirement:
  5.430° from the measured stop is feasible and 5.469° is not. The boundary solution reaches
  +98.48° wrist flex against a measured +103.91° stop. Runtime limit authority now uses
  the URDF/model range as the generic fallback/reference, but a calibrated real arm follows
  its measured pose-joint travel with a 1° inset from each mechanical stop. This preserves
  nearly all measured range while keeping normal commands off the stop itself. Validate the
  normal runtime preflight and one supervised powered replay before treating the extended
  calibrated range as physically validated. Then use the paper-height sweep only as a
  separate geometry comparison. The
  first 5 mm sweep found 130 mm (+23 mm from
  the measured 107 mm reference) fully feasible, while candidates through 125 mm remained
  wrist-flex limited. Treat 130 mm as extrapolated calibration geometry until separately
  validated. Inspect its planned joint/encoder diagnostics before any powered use. The severe
  visible shake remains a separate
  unresolved issue; after planning is feasible, inspect planned
  joint derivatives, encoder quantization, following error, and cycle timing before changing
  motor tuning.
- [ ] Complete the guided teleop-versus-programmed motion-quality study on the physical
  follower. Capture a marked slow teleop reference, guarded replay of its exact accepted
  arm-joint commands, and the same 8 deg/s saved-pose route at both 50 Hz and 20 Hz. Compare
  raw encoder-step distributions, command-versus-feedback lag, deceleration behavior, and
  folded/Sleep geometry before selecting the next control change. Physical 20 Hz versus 50 Hz
  comparison showed no large smoothness difference, while exact deterministic replay of the
  recorded teleop command sequence completed successfully and retained the visibly less-
  mechanical/smoother character of teleop. This makes generated joint-command structure the
  leading hypothesis, especially during the folded return to Sleep. Quantized-target
  coalescing has looked at most marginally better and is not merge-ready on that evidence alone.
  Keep both joint execution strategies available while testing: the existing `streamed`
  host trajectory and the experimental `final_target` one-write endpoint mode. Run the
  supervised same-route comparison, with special attention to Right -> Sleep, before choosing
  a production default or removing either implementation. The first final-target physical
  attempt completed Sleep but was stopped on Sleep -> Overhead by an invalid cross-joint
  phase-coupling guard (0.308 rad vs 0.300 rad). That guard has been removed while preserving
  per-joint corridor/overshoot, reverse-motion, effort/fault, cancellation, timeout, and
  settle checks. Because the one-shot servos may trace an asynchronous joint combination
  rather than the synchronized host plan, full-workspace/saved-pose execution now validates
  the accumulated measured intermediate path on every monitor cycle using the measured-start
  workspace policy. The corrected final-target-only rerun completed the full route but still
  rocked substantially, so repeated host micro-waypoints are not the primary cause. Retain
  both execution modes for diagnosis. Faster requested speed is somewhat smoother overall
  and higher acceleration adds little beyond that, while severe rocking returns during the
  final slowdown into canonical Sleep. Before deeper servo tuning, isolate wrist
  orientation/fold order with a physically taught wrist angle: hold canonical Sleep, relax
  only wrist_flex, hand-place it, relatch/save the measured pose, then compare direct canonical
  Sleep, direct taught-wrist Sleep, and a staged final wrist-only fold from RIGHT. Return to
  teleop-derived command shape versus programmed low-speed servo behavior after that result.
  The operator-defined `sleep2` A/B showed substantially smoother settling than the
  historical Sleep pose, confirming a large geometry component. Promote that result into a
  portable calibration-relative semantic: default Sleep uses wrist_flex at 75% of its
  executable range, while `sleep_up` preserves the historical lower-limit wrist fold.
  Re-run the standard route with the new default Sleep on hardware before merging. A 40
  deg/s, 250 deg/s² attempt showed that the old backend 250/20 Feetech profile could no longer
  track the faster validated host trajectory and tripped following-error before the A/B.
  Default streamed planned joint motion now uses the same responsive 0/254 tracking profile
  as teleoperation while retaining the host speed/acceleration ceilings and safety guards.
  The 40/250 Sleep-vs-sleep_up rerun completed cleanly after removing the hidden servo
  throttle, and 80/500 subsequently completed with a large subjective smoothness improvement.
  GUI teleoperation then isolated a different wrist-specific issue: 20 Hz Medium tracking
  works, while the former Fast 1000 deg/s² wrist-flex response could command a reversal
  faster than the physical wrist braked, producing +0.057 rad of carry-through and a strict
  opposite-direction stop. Fast is now per-joint: wrist_flex retains 100 deg/s speed but uses
  500 deg/s² acceleration, while the other joints retain 100/1000. Live-stream direction
  monitoring also permits only a 100 ms non-growing braking carry-through immediately after
  a recent genuine command reversal, capped at 0.10 rad cumulative wrong-way travel;
  planned motion remains strict. Validate isolated wrist
  reversals at 20 Hz with no payload/gripper mirroring before considering this motion-quality
  branch merge-ready.
- [ ] Resolve any remaining physical/model Cartesian-direction mismatch before broader
  low-speed linear paths/tolerance validation.
- [ ] Finish guarded leader-to-follower streaming validation. Medium at 20 Hz has passed
  supervised use; next validate Fast · wrist-aware wrist reversals, STOP, leader-readout
  loss, and stream safety trips with gripper mirroring disabled, then separately validate
  gripper mirroring/contact at Normal gripper speed. The pen overload showed that gripper
  object state must not be conflated with arm-stream dynamics.
- [ ] Validate sequence execution and edited motion primitives on hardware.
- [ ] Validate the selected USB camera on the target workstation, confirm negotiated
  resolution/FPS/FourCC, live GUI preview in Camera and Teleoperation, and still capture.
- [ ] Tag the first hardware-validated alpha release.

## Later

- [ ] Revisit optional agent sandboxing after the bounded CLI/camera robot loop is
  physically validated. The bounded CLI is the robot capability boundary; a sandbox may
  further restrict shell/filesystem access but is not an architectural dependency for the
  initial experiment.
- [ ] Investigate true leader gravity compensation as a separate feature. Compare the
  Trossen leader/SDK approach before selecting a control strategy; do not couple this to
  the current position-hold parking implementation.
- [ ] Parallel gripper implementation.
- [ ] Tool assemblies with camera and gripper TCPs.
- [ ] Optional collision geometry and self-collision checks.
