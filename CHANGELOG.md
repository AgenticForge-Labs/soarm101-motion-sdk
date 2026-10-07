# Changelog

## Guided desktop setup (locally validated)

- Compact arm readiness cards; calibration and diagnostics open on demand inside Setup.
- Optional remembered guidance, readable light/dark colors, responsive calibration gauges, and explicit offline model labeling.
- Validated portable setup backup/restore with previous-file recovery and no hardware I/O.

## Model visualization (software validated)

- Corrected the GUI stock-gripper schematic to show a fixed finger and rotating jaw
  using the official model pivot/travel. Added stable scale, standard orthographic
  views, Fit/optional Auto fit, and a millimeter ruler. The grid is explicitly model
  Z=0. Active TCP markers and numerical FK now share the measured joint snapshot.
- Recorded the nominal-model comparison and remaining physical geometry/TCP and
  shaking investigations in PLAN.md; motion/calibration policy is unchanged.

## Unreleased
- Made the persistent GUI robot view live during commanded motion without adding a competing
  hardware poller. The MotionController and stock gripper publish motion-owned measured
  feedback so the GUI animates measured joints, model-estimated TCP readouts, and jaw aperture
  while normal full-state polling is paused. Combined recorded/alignment paths sample gripper
  state only at existing controller feedback checkpoints. Visualization callbacks are
  observational and cannot affect motion.
- Added a pluggable self-contained `soarm101 agent sandbox` workflow for constrained
  autonomous robot operation without Forge-Bench or another harness. Hermes/OpenRouter and
  Codex are first-class adapters behind the same OpenShell, `robotctl`, broker, authority,
  and Motion SDK safety boundary. Codex supports OpenAI Platform API-key mode
  (`OPENAI_API_KEY`), `--auth installed` to reuse a one-run copy of the host's existing
  file-backed ChatGPT Codex login without mounting or modifying the host Codex home, and an
  optional separate SDK-owned `--auth chatgpt` device-login mode. `sandbox doctor` checks
  readiness, `sandbox setup` builds/updates adapter assets, and `sandbox run --read-only`
  provides a hard no-motion validation path that does not require a motion-authority lease.
  Read-only validation now fails unless post-handoff broker evidence shows capabilities/state
  inspection plus a SHA-verified retained capture for every configured camera, with no motion
  action reaching the broker. OpenShell-compatible CLI harnesses beyond the packaged adapters
  use a harness-neutral validation task and can use a strict JSON manifest selecting an
  existing image/provider and direct argv without changing the robot execution path. Full
  runs require pre-existing human authority; the unrestricted SDK, serial/camera devices,
  calibration files, Docker socket, SSH material, and unrelated host files are not exposed to
  the reasoning sandbox.
- Added GUI **Tracking response** presets for leader→follower teleoperation while keeping
  stream cadence independent: Slow is 0.6 rad/s and 3.0 rad/s², Medium preserves the
  previous 1.2 rad/s and 6.0 rad/s² behavior and remains the default, and Fast is
  wrist-aware. Fast allows 100 deg/s on all five joints and 1000 deg/s² on the four
  non-wrist-flex joints, while wrist_flex is capped at 500 deg/s² after a physical Fast run
  showed +0.057 rad of residual motion in the prior direction during a 1000 deg/s² reversal.
  The GUI limiter and deterministic MotionController both enforce the same per-joint limits.
  Live streams additionally distinguish a recent genuine commanded reversal from runaway
  motion: at most 100 ms of non-growing carry-through is permitted while the servo brakes,
  capped at 0.10 rad cumulative wrong-way travel; stale, growing, persistent, or over-cap
  wrong-way motion still stops the stream. Planned motion keeps the strict direction rule.
- Fixed exact human-unit envelope values such as 100 deg/s / 1000 deg/s² being rejected
  by a few floating-point ULPs after conversion to SI. Boundary-equivalent values now clamp
  to the configured ceiling while genuinely larger requests remain rejected.
- Investigated an STS3215 gripper overload observed after Sleep. Follow-up showed the
  gripper was holding a pen, so the overload is evidence that Sleep's commanded close is
  not object-aware rather than evidence against the calibration-derived 1° stop inset.
  Sleep therefore retains the calibration-derived target; operator/agent documentation now
  warns that the gripper close is a consequential tool action when an object is already held.
- Replaced the original undocumented host motion ceilings with an explicit human-facing
  100/1000 envelope: joints 100 deg/s and 1000 deg/s^2, TCP translation 100 mm/s and
  1000 mm/s^2, and TCP/tool orientation 100 deg/s and 1000 deg/s^2. Ordinary motion
  defaults remain conservative. Added SDK human-unit construction/reporting, a
  `soarm101 motion-envelope` command, degree-based `move-joints` dynamics flags, and
  matching CLI envelope overrides. The broker owns the trusted host envelope and propagates
  it to bounded agent subprocesses; `robotctl` cannot widen it. Cartesian IK-generated
  joint trajectories are now validated against the absolute joint envelope rather than the
  slower ordinary defaults.
- Removed the hidden slow Feetech profile from default host-streamed joint plans.
  Planned joint/saved-pose/Sleep trajectories now use the teleoperation-style responsive
  actuator profile (Goal_Velocity=0, acceleration=254) unless explicitly overridden, so the
  validated host trajectory owns requested speed/acceleration. This fixes a 40 deg/s,
  250 deg/s² hardware case where wrist_flex lagged the host by 0.305 rad and correctly
  tripped the unchanged 0.300 rad following-error guard. Recorded arm trajectory replay now
  uses the same 0/254 tracking profile, and experimental final-target execution keeps its
  synchronized per-joint speed calculation while defaulting servo acceleration to 254.
- Changed the calibration-relative default Sleep wrist geometry after physical A/B testing:
  `wrist_flex` now targets 75% of its executable range
  (`upper - 0.25 * (upper - lower)`). The historical wrist-at-lower-limit fold is preserved
  as `sleep_up` / `move_sleep_up()`, with operator CLI, bounded-agent CLI, broker, and
  `robotctl` access. Existing `sleep` callers automatically use the smoother posture.
- Added a simple `sleep2` hardware diagnostic: capture the current measured arm pose before
  any commanded motion, save it as the named pose `sleep2`, then compare RIGHT -> canonical
  Sleep against RIGHT -> sleep2 at identical dynamics. The test uses the same narrow
  Sleep-family coarse-self-clearance exception while retaining other workspace preflight and
  normal streamed runtime guards.
- Added a supervised Sleep-geometry diagnostic that now teaches the experimental wrist
  position physically: canonical Sleep holds the arm, only wrist_flex is relaxed for manual
  placement, the measured wrist is range/drift checked and safely relatched, and the pose is
  saved locally. The retest compares direct canonical Sleep, direct taught-wrist Sleep, and a
  staged wrist-only final fold from the same RIGHT pose. The Sleep-family diagnostic omits
  only the known coarse self-clearance heuristic while preserving floor/reach/base preflight
  and normal runtime guards.
- Added an experimental `final_target` execution mode for validated joint-space moves
  alongside the existing default `streamed` mode. Both modes build the same joint plan;
  final-target writes one synchronized endpoint command per leg and monitors guarded progress
  to settle rather than sending host micro-waypoints. Cartesian linear motion is unchanged.
  Added `scripts/run_joint_execution_comparison.sh` for a supervised same-route A/B test.
- Corrected the experimental final-target monitor after the first physical run showed that
  cross-joint phase matching falsely rejected normal asynchronous servo progress. Final-target
  now keeps fault/effort, reverse-motion, per-joint corridor/overshoot, timeout/cancellation,
  and settle guards without requiring joints to remain synchronized in phase. Full-workspace
  and saved-pose one-shot moves also validate the accumulated measured intermediate path with
  the measured-start workspace policy, so asynchronous physical motion is not assumed to
  follow the synchronized host plan. The comparison runner accepts `final_target` as a
  single-mode rerun.
- Added a guided teleop-versus-programmed motion-quality study for the low-speed shake
  investigation. It marks/extracts a slow GUI teleop reference, can replay the exact accepted
  arm-joint command sequence through guarded streaming, runs the same saved-pose route at 50 Hz
  and 20 Hz without per-leg prompts, and packages the evidence for review. The reusable passive
  backend tracer records existing joint commands, raw encoder goals, natural feedback reads,
  TCP positions, effective servo parameters, hardware-state checks, and route markers without
  adding motion-time bus polling; detailed teleop frames now include raw encoder targets and
  monotonic timing.
- Added a safe post-study exact-teleop replay runner. When the guided run ends too far from
  the first recorded teleop command for immediate streaming, the runner performs a normal
  guarded pre-positioning move with full workspace checks, verifies measured arrival, then
  reuses the original GUI session's recorded teleop speed, acceleration, command-step, and
  following-error limits to preflight and replay the exact captured sequence. Missing or
  invalid recorded settings fail closed. Success or safety-refusal evidence is appended to
  the same study archive.
- Planned calibrated motion now suppresses redundant intermediate servo writes when adjacent
  continuous joint samples resolve to the exact same five encoder targets. Host timing,
  monitoring, speed/acceleration validation, and the exact final sample are unchanged. This
  targets low-speed quantization chatter without introducing a new PID or servo-profile
  heuristic; physical smoothness validation remains pending.
- Added a narrow host-side HTTP/JSON broker for isolated reasoning agents. The broker
  serializes an explicit allowlist of bounded agent actions, delegates to the existing
  `soarm101 agent` CLI, requires a bearer token, records JSONL request evidence, and
  returns camera captures as bytes plus SHA-256 so sandboxes need no host camera mount.
  Added a standard-library-only `robotctl` client intended to be copied into OpenShell or
  other isolated workers without installing the unrestricted Motion SDK.

- Fixed boundary-valid joint moves (including calibrated Sleep) that could fail when the
  trajectory planner reconstructed the final sample one floating-point ULP beyond an exact
  effective joint-limit target. Joint planning now preserves the already-validated start and
  target samples exactly; calibrated stop margins and limit guards are unchanged.
- Added bounded single-joint relative agent adjustments (one named joint, max 30° per
  command) and tool-frame translation through the existing `agent jog` capability.
  Agent tool-frame jogs use the current gripper/TCP axes while retaining the same physical
  displacement/height policy and normal SDK motion guards.
- Fixed the general `move-joints` CLI so a successful physical joint move remains
  torque-held after process exit instead of dropping the arm on disconnect.

- Added a bounded `soarm101 agent ...` robot/camera interface for external reasoning
  agents. Human-interactive `agent arm` parks the follower and creates a time-limited
  lease bound to robot/calibration identity; non-interactive arming fails closed.
- Agent motion exposes only `agent_*` saved poses, calibration-inset gripper open/close,
  calibrated Sleep, STOP/HOLD, and world/model-frame translational jogs. Named
  `overhead`/`wrist` cameras are available through the same deterministic capture layer.
- Agent jogs additionally use the matching measured workspace transform as safety evidence:
  requested physical displacement is limited to 50 mm above 100 mm physical height and
  10 mm at or below 100 mm. Targets must remain at least 10 mm above the calibrated ground
  plane, preserving clearance for ordinary joint settle/model error observed during physical
  validation. `agent capabilities` now also exposes human direction guidance derived from
  the same paper/workspace calibration:
  left/right, forward/back, and up/down are reported as model/world XYZ deltas per physical
  millimeter, so reasoning agents can use the existing `agent jog` command without guessing
  model-axis signs.
- Physical bounded-agent validation on 2026-10-03 confirmed human-interactive arming,
  calibration-bound lease expiry/fail-closed behavior, both named camera captures, saved-pose
  departure from Sleep, persistent hold, >100 mm and <=100 mm jog request ceilings,
  STOP/HOLD without authority, and ENTER-confirmed human relax. Saved-pose joint transit
  remained visibly shaky while moving but became stable once holding; small jogs also showed
  a few millimeters of achieved workspace-position difference from the policy-predicted
  target, motivating the 10 mm ground-plane margin rather than weakening motion guards.
- Added an opt-in object-to-container robot/camera skill with a fresh-overhead-image
  completion contract under `agent-as-code/skills/robot-camera/`.
- Joint-space motion now accepts the physically measured starting state as the starting
  authority for the coarse centerline self-clearance heuristic. If the arm is already inside
  that generic envelope, motion may proceed only while modeled self-clearance does not worsen;
  it no longer has to fully exit the envelope merely to make a valid improving move. Once
  normal clearance is reached, strict self-clearance validation resumes and re-entry is
  rejected. Other workspace and runtime motion guards remain unchanged. This fixes normal
  folded/resting poses being rejected at workspace path sample 0 during replay pre-positioning.
- Physical `pose go` now preserves torque hold after the CLI disconnects instead of
  automatically relaxing at command exit. `soarm101 relax` requires explicit ENTER
  confirmation. Sleep likewise holds after reaching the folded posture and only relaxes after
  the operator presses ENTER.
- Hardware validation of the endpoint-seeded reverse fallback still failed safely on
  A_UP->B_UP. The forward solve bottomed out at 0.648 mm and the reverse solve at
  0.646 mm against the unchanged 0.5 mm Cartesian tolerance, while both endpoints still
  preflighted at 0.00 mm. Because both directions converged to essentially the same miss,
  the next question is geometric reachability/singularity/joint-limit interaction rather
  than another directional solver heuristic.
- Paper replay now performs read-only full-segment preflight for
  A_UP->B_UP->C_UP->D_UP->CENTER_UP before offering powered traversal. A failing IK sample
  reports command-rate sample index, line progress, target XYZ, signed XYZ residual, best
  joint solution, positional Jacobian conditioning, and nearest effective joint-limit
  margin. This prevents repeated shaky powered entry motion when a later elevated segment
  is already known to be unplannable.
- Added `soarm101 limits --json` as a hardware-free view of saved mechanical calibration
  ranges, nominal model limits, the arm-specific executable range, the active calibrated
  extension stop margin, and the configured coarse Cartesian envelope.
- Added a read-only paper `--limit-compare-only` diagnostic. It compares the saved
  reference-height Cartesian path under the unchanged executable model/calibration
  intersection versus calibration-derived joint bounds inset from the measured mechanical
  stops by `--calibration-stop-margin-deg`. Explicit diagnostic IK bounds are accepted
  only by read-only planning and may never exceed the active calibration; executable
  `move-linear`/joint motion limits are unchanged.
- The current arm calibration measured wider stop-to-stop travel than the nominal model
  on every pose joint: shoulder pan ±121.1° vs ±110°, shoulder lift ±105.1° vs ±100°,
  elbow flex ±97.0° vs about ±96.8°, wrist flex ±103.9° vs ±95°, and wrist roll
  ±168.8° vs roughly -157.2°/+162.8°. This means the prior +95° wrist-flex failure was
  a nominal model boundary, not the measured physical stop. Hardware-side read-only
  comparison at the saved 107 mm paper height confirmed the distinction: the normal
  model/effective bounds failed A_UP->B_UP at the +95° wrist-flex cap, while calibration-
  derived bounds inset 3.0° from the measured stops planned all four straight Cartesian
  segments successfully. The feasible path used wrist flex up to +100.91°, exactly the
  +103.91° measured stop minus the 3° inset, so the margin is binding. Added a torque-off
  binary-search diagnostic to find the largest measured-stop margin that still keeps the
  107 mm path feasible. The completed search bounded that transition between 5.430° feasible
  and 5.469° infeasible; the boundary solution used wrist flex through +98.48°, leaving
  5.43° to the measured +103.91° stop. Runtime limit policy now treats the official URDF
  limits as the generic fallback/reference while a calibrated real arm uses its measured
  pose-joint travel with a 1° inset from each mechanical stop. A narrower calibration
  remains authoritative. This gives the current arm nearly all of its measured travel,
  including wrist flex to about ±102.91°. Endpoint IK, joint motion, Cartesian planning,
  live streaming, and the limits CLI share this resolver.
- Sleep closes the stock gripper after the arm fold completes. The close target is
  derived from the active gripper calibration and defaults to 1° inside the calibrated
  closed mechanical stop rather than normalized 0.0 at the stop itself. The conversion is
  drive-direction independent, and `soarm101 limits --json` reports the normalized/raw
  Sleep gripper target for read-only inspection.
- Added a calibration-derived Sleep posture for demos and power-down preparation.
  Sleep is computed from the active executable limits: shoulder pan midpoint, shoulder lift
  lower limit, elbow flex upper limit, wrist flex lower limit, and wrist roll midpoint.
  On a calibrated arm those endpoint limits are already 1° inside the measured mechanical
  stops. It is exposed through `SOARM101.get_sleep_joint_positions()`,
  `SOARM101.move_sleep()`, `soarm101 sleep --yes`, and a GUI **Go Sleep** control.
  `soarm101 limits --json` reports the exact derived Sleep pose without moving hardware.
  Sleep is never commanded automatically on connect or torque enable. The dedicated Sleep
  primitive skips only the generic coarse workspace-geometry check because the designed
  folded posture violates the generic 25 mm link-centerline self-clearance heuristic; normal
  joint motion still uses that check, and calibrated joint, trajectory, following-error,
  effort, fault, communication, and completion guards remain active.
- Added `--height-sweep-only` for replay diagnostics. It keeps torque disabled and searches
  the calibrated workspace Z range for the nearest height whose endpoints and complete
  straight paper path are feasible under the unchanged model/calibration joint limits.
  Explicit-start Cartesian planning is now genuinely read-only and no longer requires
  torque to be enabled. On the saved paper calibration, the nearest-first 5 mm sweep
  continued to hit the wrist-flex upper limit through 125 mm and found all four straight
  segments feasible at 130 mm (+23 mm from the measured 107 mm reference). Because 130 mm
  is above the measured reference, it remains extrapolated workspace geometry rather than
  powered-motion validation.
- Feasible height-sweep results now print per-segment planned joint step/speed/acceleration/
  jerk plus per-joint jerk, direction reversals, encoder zero-delta fraction, and maximum
  encoder step. These are read-only diagnostics for the unresolved visible shake.


- Hardware replay after #74 remained very shaky and A_UP->B_UP still failed safely during
  pre-motion planning, now at 0.794 mm against the unchanged 0.5 mm Cartesian tolerance.
  Review of the public managed controller confirmed that real Cartesian execution was
  already using the intended teleoperation-style `speed_raw=0`,
  `acceleration_raw=254` profile, so the visible shake remains unresolved and this change
  does not introduce another servo-profile experiment.
- Position-only Cartesian planning now has an endpoint-seeded reverse fallback. If forward
  sequential IK hits a numerical pocket, the planner solves the same Cartesian samples
  backward from a reachable target solution and accepts the result only if it reconnects
  continuously to the measured start. The paper workflow reuses its exact read-only
  endpoint-preflight joint solution as that boundary-condition seed. Every sample still
  must satisfy the same hard Cartesian tolerance, joint limits, continuity, dynamic, and
  runtime safety checks.


- Superseded the #73 launch-only smoothness experiment after physical replay remained
  shaky and A_UP->B_UP again failed during read-only planning with a 0.812 mm intermediate
  IK miss against the unchanged 0.5 mm tolerance. Cartesian timing returns to the symmetric
  half-cosine profile used before #73; the validated teleoperation-style Feetech tracking
  profile remains unchanged.
- Hardened numerical IK against soft-regularization false misses. If continuity/joint-center
  penalties leave a candidate outside the hard task tolerance, the solver performs a
  task-space-only bounded refinement from the regularized candidates and still requires the
  original tolerance.
- Added Cartesian-constrained smoothing for position-only paths. A five-tap filter creates
  smoother joint seeds, each interior Cartesian sample is re-solved at the unchanged
  tolerance, and the refined sequence is accepted only when discrete joint jerk decreases.
  Cartesian geometry, endpoint targets, workspace policy, host cadence, and servo authority
  are unchanged.

- Refined Cartesian launch smoothness after hardware replay became faster/error-free
  but remained visibly shakier at the beginning of each line than near the endpoint. The
  host profile now uses bounded constant acceleration from rest, optional cruise, and the
  existing half-cosine deceleration. At the paper validation settings (20 Hz, 20 mm/s,
  100 mm/s²), the first 50 ms Cartesian increment increases from about 0.021 mm to
  0.125 mm while respecting the same speed/acceleration ceilings. The validated
  teleoperation servo profile (speed_raw=0, acceleration_raw=254), Cartesian geometry,
  endpoint IK, startup clearance, cadence, and workspace policy are unchanged.

- Restored the supervised elevated-paper experiment to its intended purpose: validating
  Cartesian `move_linear()` at one calibrated physical height. The newer workspace
  leveling and true paper-center geometry remain, but A_UP→B_UP→C_UP→D_UP→CENTER_UP
  is again executed as Cartesian linear segments rather than the temporary joint-space
  endpoint replay introduced in #67–#71.
- Planned Cartesian Feetech tracking now matches the physically smooth leader/follower
  strategy: host-side trajectory generation remains the sole speed/acceleration shaper,
  while each streamed setpoint uses `speed_raw=0` and `acceleration_raw=254`.
  This supersedes the calibrated per-sample proportional servo-speed caps, which added a
  second quantized trajectory on top of the Cartesian IK stream and are the leading
  explanation for the observed stick-slip/shaking.
- Added regressions requiring calibrated `move_linear()` to retain the teleoperation
  servo profile and requiring the paper perimeter to call `move_linear()`, not
  `move_joints()`.

- Fixed paper joint replay after the first synchronized-servo hardware test reached
  A_UP preflight but was rejected before motion by the generic model-frame workspace floor
  at sample 0 (model TCP z=-2 mm), despite the calibrated paper path reporting a safe
  physical workspace Z minimum of 23.3 mm. Guarded `move_joints()` now exposes the same
  explicit `full / target_only / off` workspace-check modes as `move_linear()`, with
  `full` remaining the default. The paper workflow uses `target_only` only after its
  calibrated full-path FK check, preserving a generic destination sanity check without
  allowing the known-invalid model table floor to override measured workspace authority.

- Paper joint-space replay now enables calibrated synchronized servo arrival on each
  20 Hz planned sample. The joint trajectory and timing are unchanged; only the servo-side
  pacing changes from fixed unrestricted speed to proportional per-joint speed limits so
  all joints target the next sample on the same arrival horizon. This is a targeted
  smoothness change built on the physically successful joint-space replay.

- Fixed CENTER_UP geometry in the supervised paper replay. The previous target came
  from averaging the four corner joint states and applying FK, which is not the physical
  midpoint because FK is nonlinear. CENTER_UP is now generated directly from the calibrated
  paper frame at (physical_width/2, physical_height/2, reference_height); the averaged-joint
  pose is retained only as an IK seed.

- Startup clearance acceptance is now based on measured rise rather than Cartesian
  endpoint shortfall. Replay still commands 20 mm by default, but requires at least
  10 mm measured workspace-Z increase before paper travel. Hardware evidence motivating
  the threshold is explicit: the earlier dragging case rose only about 5.4 mm, while the
  later visually acceptable startup rose about 14.0 mm. The legacy
  `--startup-height-tolerance-mm` option remains available as an explicit compatibility
  override.

- Hardware replay reached A_UP but the straight A_UP->B_UP Cartesian path still failed
  at an intermediate pose with 0.782 mm residual against the unchanged 0.5 mm IK
  tolerance, while both endpoints preflighted at 0.00 mm. Elevated paper traversal now
  reuses the endpoint IK joint solutions with smooth joint-space interpolation and the
  teleoperation servo profile. Before each move, dense FK sampling rejects any joint-space
  locus that would dip more than 5 mm below the lower endpoint in calibrated workspace Z.
  The one-shot 20 mm Cartesian startup clearance is unchanged.

- Paper replay now uses one 20 mm calibrated-Z startup clearance move by default,
  checks the measured endpoint against the existing 5 mm tolerance, and then follows
  A_UP -> B_UP -> C_UP -> D_UP -> CENTER_UP. The D_UP sample remains the workspace-height
  reference but is no longer repeated as the first powered target.

- Hardware replay after #64 still moved laterally toward the base before rising and
  remained much slower/jerkier than teleoperation. Cartesian planning now uses smooth
  half-cosine acceleration/deceleration ramps with constant-speed cruise when distance
  permits; requested linear speed is a real cruise ceiling instead of a minimum-jerk peak.
  Position-only motion no longer incurs an irrelevant orientation-time penalty.
- Calibrated Feetech Cartesian execution now derives a per-joint position-mode speed for
  every synchronous write from the planned encoder-tick increment and command interval.
  This keeps lightly loaded joints from racing at unrestricted speed while gravity-loaded
  joints lag behind the same Cartesian sample.
- Paper startup now climbs to transport height in separately preflighted, settled, and
  remeasured 10 mm calibrated-Z stages by default, bounding low-height tracking error
  before any lateral traversal.

- Hardware replay showed that a nominal 10 mm startup clearance reached only 5.8 mm
  measured workspace Z and still allowed the tool to drag the paper. Paper replay now
  lifts straight to at least the full reference/transport height before lateral travel,
  verifies measured workspace Z within 5 mm, and retries up to three separately
  preflighted lift corrections before failing closed. The supervised paper workflow also
  uses the known-smooth 20 Hz teleoperation host cadence while leaving the SDK-wide 50 Hz
  planned-motion default unchanged.
- Intermediate command-rate Cartesian IK now retries with multi-start when a sequential
  single-start solve misses the existing 0.5 mm tolerance. Hardware exposed this as a
  0.556 mm B-segment miss; the retry improves branch recovery without loosening tolerance.

- Fixed paper replay startup from low/resting poses where the calibrated physical-Z
  clearance target was valid but the generic model-frame workspace floor rejected it
  (for example model TCP z=-11 mm). The startup lift now receives a dedicated calibrated-
  workspace preflight that requires fixed workspace X/Y, nondecreasing physical Z,
  sequential IK continuity, and effective joint limits. Only that verified clearance move
  bypasses the generic model-Z floor; the normal runtime joint, step, speed/acceleration,
  following-error, effort, fault, communication, and timing guards remain active.

- Superseded the temporary multi-height teaching approach from #61. The original
  A/B/C/D + one physically measured D_UP workflow remains authoritative. Hardware showed
  that the reachable FK replay landed at ~30 mm workspace Z at A/B and ~107 mm at C/D;
  replay now preserves each reachable endpoint's calibrated workspace X/Y and explicitly
  sets workspace Z to the single measured reference height before mapping back to model
  coordinates. Every corrected endpoint is read-only IK-preflighted before motion.
  Replay also preflights and performs a 10 mm calibrated workspace-Z clearance lift before
  the first long move from a low/resting pose. No additional elevated teaching is required.

- Cartesian `move_linear()` no longer solves sparse IK waypoints and then linearly
  interpolates between those joint-space knots. Hardware testing showed that reducing the
  knot spacing to 1 mm did not remove visible shaking. Linear planning now applies the
  minimum-jerk progress law in Cartesian space and solves sequential IK directly at each
  host command-rate sample. Joint limits, per-step limits, velocity/acceleration retiming,
  workspace validation, cached-plan start validation, and the responsive servo profile
  remain in force.
- Hardware replay after #58 confirmed that translating the measured D→UP Cartesian
  displacement still left B_UP 10.09 mm outside IK tolerance. The supervised paper
  traversal now restores the previously preflighted construction: apply the taught D→UP
  **joint** delta to each taught corner joint pose, verify limits, then use FK to define
  known-reachable Cartesian endpoints. Only D_UP has direct physical-height evidence;
  affine-inverse heights for the other endpoints are reported as diagnostics rather than
  claimed as an exact constant physical height. Exact constant-height validation requires
  additional elevated physical correspondences.

- Corrected the constant-height paper replay after hardware preflight showed that the
  global affine transform placed B_UP 10.09 mm outside IK tolerance. The fixed supervised
  traversal now anchors A/B/C/D to their directly taught model positions and adds the
  directly measured D→UP displacement to each corner; CENTER_UP is the mean of those
  elevated measured corners. This preserves the trained physical lift without letting
  affine fit residuals move measured endpoints. The affine transform remains diagnostic
  workspace evidence. The 1 mm Cartesian IK waypoint spacing remains in effect.

- Cartesian `move_linear()` now uses the same responsive Feetech servo-side profile as
  live teleoperation (speed_raw=0 / unrestricted, acceleration_raw=254). The host-side
  50 Hz trajectory remains authoritative for Cartesian/joint speed and acceleration.
  This removes the previous second slow servo trajectory (250/20) that could make a
  stream of smooth waypoints lag and catch up in visible bursts.

- Relaxed only the supervised paper-linear experiment's completion criterion to 3.0°
  per joint with an 8 s settle window (SDK-wide defaults remain unchanged). The previous
  P=32 hardware replay reached within 2.18° but was still rejected by the global 1.43°
  precision criterion.
- Settle timeouts now identify the worst joint, report all per-joint target-minus-measured
  errors, and include best-effort live motor voltage/current/moving/status diagnostics.

- Restored the STS3215 follower position P gain to the factory value 32 instead of 16.
  Hardware testing showed a slow/shaky gravity-loaded Cartesian climb that stopped about
  0.071 rad from target, and an upstream SO-arm report independently documents a P=16
  static-friction/deadband problem that is resolved by P=32.
- Paper Cartesian validation now stops/holds on motion failure and waits for the operator
  to support/release the arm instead of immediately relaxing and dropping it.

- Reworked the paper experiment around its actual purpose: validating Cartesian
  `move_linear()`. After UP teaching, the arm now counts down and holds that pose;
  `--replay` can reuse the saved teaching from an ordinary resting pose without
  reteaching. Elevated endpoints are generated from known-reachable FK configurations
  inferred from taught corner joints plus the taught D→UP joint delta, while the segments
  themselves remain Cartesian linear moves. The supervised paper workflow now uses
  target-only coarse workspace checks on each segment, retaining the joint/dynamic/
  following-error/fault/effort/communication/timeout guard stack.

- Refined the supervised paper traversal after hardware preflight showed affine-
  extrapolated B_UP was 14 mm outside the IK tolerance. Powered targets now use the
  directly measured A/B/C/D model positions translated by the directly measured D→UP
  displacement; CENTER_UP is the mean elevated corner. Preflight also tries measured-
  pose-derived multi-seed IK branches before any torque-on motion.

- Corrected paper workspace measurement acceptance: D→UP wrist/tool-orientation change
  is now diagnostic only. With a 5-DOF arm and the fixed fingertip as the probe, wrist
  rotation may be required to reach the measured UP point. Table fit, affine residual,
  mapping conditioning, and plausible UP scale remain the acceptance gates; saved
  workspaces remain unvalidated for autonomous motion.

- Fixed the managed Feetech torque-enable precheck to actually implement the documented
  calibrated-endpoint tolerance: a measured relaxed position up to 8 encoder ticks beyond
  an EEPROM limit is now clamped inward for the startup latch, while larger violations
  still fail before any goal or torque write. Commanded motion remains bounded by the
  calibrated limits.

- Superseded the paper hover / base-Z floor-recovery experiment after physical hardware
  showed that a numerically valid model +Z move could travel laterally into the table.
  Both paper workflows are now measurement/calibration only: the three-corner script
  reports geometry without motion, while the four-corner script adds a manually measured
  physical-UP point and persists a motor-calibration-bound local workspace transform.
  Newly measured workspace calibrations remain unvalidated for autonomous Cartesian
  motion until a separate supervised direction test passes.

- Fixed the managed Feetech torque path to use the same bounded readback/retry recovery
  as the protocol backend for idempotent `Lock` and `Torque_Enable` writes. A single
  lost status packet during guarded enable/disable no longer aborts paper/CLI/GUI startup
  when register readback confirms the requested value; persistent communication failures
  still fail closed and torque-enable rollback keeps EEPROM locked.

- Set the OpenCV camera buffer request to two frames after hardware testing showed that a
  one-frame V4L2 buffer halved icSpring capture throughput. The GUI's one-frame latest-preview
  mailbox remains unchanged, and live diagnostics continue to show measured capture FPS and
  preview age.

- Changed named-camera GUI previews to use a one-frame latest-observation mailbox per camera
  and a 30 Hz UI refresh. Superseded display frames are discarded, with capture FPS,
  presentation age, and preview-coalescing counts shown on live camera cards. Snapshot
  capture remains on the camera worker and retains fresh-observation semantics.

- Restored 20 Hz as the practical live leader→follower teleoperation default after 10 Hz
  proved visibly stepped during physical hand-following. Follower cycle overruns remain
  measured, but teleoperation now stops on actual stale queued leader data rather than
  three nominal-period overruns alone. The existing communication, following-error,
  hardware-fault, effort, and STOP/HOLD guards remain active.
- New camera profiles default to 640×480 MJPG at 15 FPS to reduce USB bandwidth and host
  processing load; existing saved camera profiles are preserved unchanged.

- Made Feetech torque/lock control writes tolerant of a single corrupted or missing status
  reply: the backend first reads the control register back, accepts the write only if the
  requested value is confirmed, otherwise retries the same idempotent write once and still
  rolls back/fails closed on persistent communication errors.

- Added explicit USB camera disconnect recovery for live GUI streams. A vanished Linux camera
  or V4L2 reopen failure after a previously live stream now enters a visible WAITING state for
  up to 30 seconds, keeps the last good frame, and automatically reopens the same saved stable
  device identity when it returns.

- Made GUI camera streams tolerant of transient USB frame drops: one missed frame no longer
  stops a stream, snapshots survive retry, repeated misses trigger device reopen, and terminal
  open failures remain explicit.
- Tightened the multi-camera dashboard with smaller preview cards, recovery-state labeling,
  friendly device names, and automatic migration from legacy `/dev/videoN` selections to
  stable Linux `/dev/v4l/by-id/` identifiers when possible.

- Made the Camera tab more compact, turned USB device selection into an explicit discovered-
  device dropdown, and added an adaptive named-camera preview area: one view fills the panel,
  two split side-by-side, and larger sets use a two-column grid.

- Increased Motion Studio button contrast using colors computed from the active Qt palette,
  added explicit primary/danger action roles, and kept disabled controls visibly button-shaped
  instead of allowing Ubuntu palette roles to collapse into the page background.

- Added one machine-local workstation profile shared by GUI, CLI, and agents for follower/
  leader ports, robot/calibration identities, and named cameras.
- Added named multi-camera GUI sessions: profiles such as `overhead` and `wrist` can run
  concurrently, while Camera and Teleoperation choose which shared stream to display.
- Added named camera CLI operations, `camera capture --all`, workstation inspection/arm
  configuration, and stable Linux `/dev/v4l/by-id/*-video-index0` discovery when available.
- Migrated the legacy single-camera configuration into the workstation profile and removed
  duplicate hardware addressing from agent-as-code experiment setup.
- Simplified high-density GUI guidance into contextual help/tooltips and strengthened the
  global push-button treatment so actions are visually distinct across tabs.

- Added a neutral `docs/agent-arm101-cli.md` tool contract for direct coding-agent and
  automation use without prescribing an agent reasoning or action policy.
- Added read-only `soarm101 ik` target solving plus JSON output for read, joint motion,
  Cartesian jog, linear motion, and gripper commands. Read/motion commands now support
  `--simulation` consistently for software-only agent checks.
- Simplified `agent-as-code/` to machine-local robot/camera setup and multi-view capture
  support; agent launchers, model selection, sandboxing, and task policy remain outside the SDK.
- Added hardware-free setup validation coverage for primary-camera selection and ordering.

- Added first-class local USB camera support shared across SDK, CLI, and GUI. Camera settings
  persist once for device, resolution, FPS, FourCC, mirroring, auto-start, and snapshot folder.
- Added a dedicated Camera tab with live preview, start/stop, device discovery, settings, and
  still capture. Teleoperation now embeds the same live stream and can capture a still without
  opening a second camera session.
- Added `soarm101 camera list`, `camera show`, `camera configure`, and `camera capture`
  for constrained agent observation loops. The camera layer returns raw frames only; perception
  and task reasoning remain outside the SDK.

- Replaced duplicated per-tab arm cards with one persistent **right-hand follower sidebar**
  outside the tab widget. It remains visible in Setup, Manual, Teleoperation, Teach /
  Record, Edit recordings, Programs, and Log.
- The solid 3D arm is now always the live follower when connected. The sidebar also shows
  all five current joint angles, full TCP XYZ/RPY, gripper position, motion/torque/fault
  status, and persistent Enable hold / STOP-HOLD / Relax controls.
- Context from the active workflow is shown only as a ghost overlay: leader in
  Teleoperation, saved pose in Teach, scrubbed recorded pose in Edit recordings, and the
  selected destination in Programs. Selecting the leader as a teaching source no longer
  replaces the primary follower model.
- Added a broader modern Qt style pass with cleaner cards, tabs, controls, spacing,
  rounded inputs/buttons, status chips, and a resizable task/sidebar workspace. The
  model/readout portion may scroll vertically on short windows while Enable hold,
  STOP/HOLD, and Relax remain pinned at the bottom of the sidebar.
- Renamed the follower torque-off sidebar state from **FREE** to **RELAXED** to avoid
  confusing it with the leader's explicit FREE/PARKED teaching state.

- Added a **Radial / shoulder-pan pattern** generator to Programs. It uses any selected
  saved position as a base, generates a start/end/increment series of pan offsets, and
  executes each row by overriding only `shoulder_pan` while all other joints inherit the
  saved pose. Generated angles are checked against shoulder-pan limits; the base pose is
  never modified.
- Generalized Program saved-position steps to support explicit joint overrides while
  rejecting Cartesian-linear replay when an override would make the stored TCP inconsistent.
- Clarified that continuous trajectory recording/replay/editing remains first-class and
  is the intended future demonstration-data path into Robo Puppeteer as well as a direct
  deterministic motion capability.

- Reframed deterministic GUI sequencing as **Programs**: users can save named positions,
  append Move/Open/Close/Custom-gripper/Wait steps, assign a per-Move speed multiplier,
  reorder the linear list, save it, and run one step or the complete program. The existing
  `MotionSequence` persistence/runner remains the compatibility layer underneath.
- Modernized the shared arm renderer with a rounded gradient canvas, floor grid, link
  shadows, clearer joints/TCP/gripper styling, and reusable ghost-pose overlays; these
  previews now feed the persistent sidebar rather than separate per-tab cards.
- Reorganized Teach / Record around saved positions first; continuous trajectory recording
  is now presented as an optional workflow for cases where the exact path/timing matters.

- Added explicit leader **FREE / PARKED** states in Manual and Teleoperation. Parking
  latches the leader's freshly measured pose and enables torque; release returns it to
  back-drivable teaching. Starting or relinking live teleoperation releases a parked leader.
- Added guarded cross-arm handoff controls: capture a fresh follower pose and move the
  leader to it, capture a fresh leader pose and move the follower to it, optionally include
  the gripper, or **Relink here — no motion** to establish a new relative teleoperation
  reference without forcing either arm into the other's pose.
- Added **Stop → Manual + park leader** for the coarse leader → fine Manual teaching
  workflow while the follower holds its current position.
- Reorganized Manual into Joint / angular and Cartesian arm modes with one persistent
  gripper tool panel. The kinematic view now stops the arm centerline at wrist roll and
  renders a schematic two-jaw gripper from the actual modeled gripper-link frame instead
  of drawing wrist-roll → TCP as a fake arm link.
- Made the GUI gripper-speed preset consistent across manual gripper moves, teleoperation
  alignment/live mirroring, Home/Rest gripper completion, sequence gripper actions, and
  recorded trajectory replay. Edit and Run surfaces now expose the same synchronized preset.

- Reworked Cartesian jogging around small, repeatable bench motion: the GUI now defaults
  to 2 mm translation steps, accepts up to 32 queued jog clicks while the current jog
  completes, executes them sequentially through the existing guarded motion path, and
  clears the queue on STOP/HOLD, relax, disconnect, or jog failure.
- Added requested-versus-achieved TCP diagnostics and a joint-center kinematic view driven
  by the same native SO-101 FK model as planning. The view now stays visible in the Manual
  workspace beside the Joints/Cartesian/Gripper controls, starts in an orthographic X/Z
  side view, can be rotated by dragging, and resets to the side view on double-click.
- Added full-path 2 mm world-axis regression tests so +X/-X, +Y/-Y, and +Z/-Z must produce
  distinct requested Cartesian displacement in simulation.
- Added a configurable 0.5 mm Cartesian IK position tolerance for planned linear paths.
- Torque enable now tolerates only a tiny calibrated-endpoint overshoot: a measured position
  up to 8 encoder ticks (about 0.7°) outside the active EEPROM limit is clamped inward to
  the limit and clearly logged before torque is enabled; larger violations still fail closed.

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
- Made leader→follower streaming rate explicit with 5/10/20/50 Hz GUI choices.
  Hardware validation proceeds 5→10→20 Hz, 10 Hz is the conservative default, and
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
