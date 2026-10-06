# Architecture

Dependency direction is strict:

```text
applications -> SOARM101 -> motion/kinematics/tools -> backend -> Feetech transport
applications -> CameraCapture -> OpenCV -> local USB/UVC camera
sandboxed agent -> robotctl -> agent broker -> bounded agent CLI -> SOARM101/CameraCapture
```

Rules:

- Basic local USB-camera discovery, named persisted capture profiles, live preview, and still-frame capture are owned here so CLI, GUI, and constrained agents share one deterministic observation surface.
- The arm has five pose joints. The stock motor-6 gripper is an `SO101Gripper` tool.
- Tools define motion-relevant TCP transforms. Tool/stage camera extrinsics, perception, tracking, OBS, and show-level capture orchestration remain higher-level concerns; this repository only owns the basic camera device session and raw frames.
- No LeRobot import exists in the runtime package.
- Hardware and simulation implement the same backend contract.
- Cartesian paths are validated before execution.
- GUI and CLI features call the same SDK operations and saved libraries; the GUI owns persistent hardware sessions rather than launching CLI subprocesses.
- Sandboxed agents may use the optional host-side agent broker. The broker is transport-only:
  it serializes an explicit HTTP/JSON allowlist and delegates to the bounded agent CLI rather
  than reimplementing motion policy. It exposes no remote arm/disarm/relax, raw servo,
  arbitrary-command, calibration, or configuration endpoint. The sandbox-side client contains
  no robot SDK and does not receive serial or camera devices.
- The GUI term **Program** is a presentation layer over the persisted `MotionSequence`
  model and `SequenceRunner`. Saved-position programs therefore do not introduce a
  second execution engine or bypass sequence provenance/safety checks.
- The GUI owns one persistent follower-status sidebar outside the task tabs. Its primary
  kinematic model is always the measured follower state when connected; active workflows
  may add a secondary leader/saved/recorded/program ghost without replacing that primary
  state.
- Kinematic GUI previews consume the same `SO101KinematicModel` used by planning; ghost
  overlays are visualization only and never authorize or execute motion.
- Physical motion artifacts carry calibration provenance and must fail closed on missing or mismatched target calibration during real-arm replay.
- Motor calibration and workspace calibration are separate authorities. Motor calibration maps encoder state to joint coordinates; machine-local workspace calibration records measured physical-workspace correspondences tied to one motor-calibration ID.
- Executable pose-joint limits are resolved once in the motion safety layer. The official
  URDF/model limits are the generic fallback/reference; when an active mechanical-stop
  calibration is present, deterministic runtime authority follows that arm-specific
  calibration and stays 1° inside each measured pose-joint stop by default. A narrower
  measured range remains authoritative. Endpoint IK, joint motion, Cartesian planning,
  live streaming, and limit reporting share this resolver. Read-only diagnostics may test
  alternate margins but do not bypass the saved calibration.
- A workspace calibration is measurement evidence until physical motion validation succeeds. The supervised paper traversal is a narrow validation exception that uses one manually measured D_UP correspondence to define the local physical workspace Z coordinate. For each replay endpoint, deterministic code preserves the endpoint's inverse-mapped workspace X/Y and replaces only workspace Z with the measured reference height, then requires read-only IK preflight before motion. Startup consists of one preflighted calibrated-workspace-Z clearance move of 20 mm by default; the measured physical/workspace rise must be at least 10 mm before paper travel. Replay then enters at A_UP and proceeds A_UP→B_UP→C_UP→D_UP→CENTER_UP. D_UP remains calibration evidence rather than an obligatory first target. The supervised elevated paper workflow is specifically a Cartesian linear-motion validation. Software levels A_UP/B_UP/C_UP/D_UP/CENTER_UP to one calibrated workspace Z, then connects those endpoints with the SDK's `move_linear()` primitive and position-only sequential IK at a 20 Hz paper-validation cadence. Because the workspace mapping is affine, the requested line between equal-workspace-Z endpoints remains constant height in calibrated workspace coordinates. Planned Cartesian execution uses the same responsive Feetech tracking profile as live teleoperation (speed_raw=0, acceleration_raw=254); deterministic host-side trajectory generation owns speed and acceleration rather than adding a second servo-side pacing trajectory. The generic model-frame floor is known to disagree with the measured table, so the separately preflighted startup lift uses calibrated workspace authority and paper segments may use the documented generic destination-only sanity check. Runtime joint/IK/dynamic/following-error/effort/fault/communication/settle/timing guards remain active. This validation is evidence for the `move_linear()` primitive on the measured setup; it does not by itself authorize broader autonomous Cartesian motion.
- A physically measured joint-space starting state is accepted as the authoritative starting
  condition for the coarse centerline self-clearance heuristic. If that measured state is
  already inside the generic self-clearance envelope, ordinary joint-space motion may proceed
  only while minimum modeled self-clearance does not worsen (within the small numerical
  tolerance); the path does not have to fully exit the envelope to be useful. If it does exit,
  strict self-clearance validation resumes immediately and re-entry is rejected. This exception
  does not weaken calibrated joint limits, model floor, TCP reach, base keep-out, command
  dynamics, following-error, effort, fault, communication, or settle checks.
- Planned calibrated motion may omit an intermediate hardware write only when the complete
  five-joint command resolves to the same encoder targets as the last sent command. The host
  schedule and monitoring clock continue unchanged, and the exact final planned sample is
  always written. This actuator-resolution de-duplication does not authorize path retiming,
  larger steps, or weaker speed/acceleration/following-error/fault/effort guards.
- Joint-space plans may be executed in either `streamed` mode (the default host-sampled
  trajectory) or the experimental `final_target` mode. Both modes build the same validated
  joint plan. `final_target` writes the endpoint once with per-joint servo speed limits
  derived from the validated duration, then observes the hardware until settle. During that
  transit, endpoint lag is expected and therefore is not treated as ordinary following error;
  deterministic monitoring instead enforces calibrated/path authority, fault/cancellation,
  unexpected-direction, per-joint start-to-target corridor/overshoot bounds, timeout, and
  final settle. Cross-joint phase matching is intentionally not a transit guard: independent
  servos may progress at different rates under gravity/load even when their speed limits are
  chosen for similar arrival time. For full-workspace and saved-pose execution, the measured
  intermediate joint configurations are appended to an observed path and validated through
  the same measured-start workspace policy on every monitor cycle. This prevents the endpoint
  mode from treating the synchronized host plan as evidence for an asynchronous physical path.
  Explicit target-only/off modes preserve their existing scope, including Sleep's deliberate
  coarse-workspace exception. This is a different execution primitive, not a bypass around
  planning or safety.
  Cartesian `move_linear()` remains host-streamed because its straight TCP path is part of
  the command contract and cannot be preserved by sending only the final joint solution.
- Planned host-streamed joint motion and live streaming share the responsive Feetech
  tracking profile: the host trajectory owns speed/acceleration shaping while the servo is
  not given a second slower velocity/acceleration cap. Default streamed joint writes therefore
  use Goal_Velocity=0 (unrestricted/max) and acceleration=254 unless a low-level caller
  explicitly overrides that profile. This does not relax host-side joint speed/acceleration,
  following-error, fault, effort, timing, or settle guards.
- Planned motion and live streaming share the core joint/rate/following-error/fault/effort safety stack, while live-stream workspace checks remain opt-in until the table frame and tool geometry are calibrated. GUI teleoperation adds named Slow/Medium/Fast response presets: Medium preserves the historical 1.2 rad/s / 6.0 rad/s² behavior, Slow halves it, and Fast is per-joint—100 deg/s is available to all five joints, the four non-wrist-flex joints may use 1000 deg/s², and `wrist_flex` is capped at 500 deg/s² based on physical reversal evidence. The GUI target limiter shapes the command toward the leader, and the same selected per-joint ceilings are passed into the deterministic MotionController stream state. Those session ceilings are themselves bounded by the configured absolute stream envelope, so a preset can narrow but never widen the hardware contract. Live-stream direction monitoring owns one additional bounded state: immediately after a recent commanded reversal it may tolerate up to 100 ms of non-growing residual motion in the prior physical direction, capped at 0.10 rad cumulative wrong-way travel. This is a streaming/braking distinction, not a relaxation of planned-motion direction validation, following-error, fault, effort, or endpoint authority.
Unprofiled guarded arm writes use the same responsive fallback (`speed_raw=0`,
`acceleration_raw=254`) so no lower layer can silently reintroduce the historical
`250/20` throttle. Tool actuators are separate contracts; the stock gripper preserves
its gentler `250/20` default pacing because contact-tool behavior is not arm-trajectory
tracking.



## Motion-envelope ownership

The Motion SDK owns one deterministic host-side motion envelope. Its current absolute
characterization ceilings are deliberately easy to reason about:

```text
joint velocity             100 deg/s
joint acceleration        1000 deg/s^2
TCP linear velocity        100 mm/s
TCP linear acceleration   1000 mm/s^2
TCP angular velocity       100 deg/s
TCP angular acceleration  1000 deg/s^2
```

Internal planner math remains SI (`rad/s`, `rad/s^2`, `m/s`, `m/s^2`). These values
are ceilings rather than default motion requests. Cartesian planning may request translation
and orientation rates independently, but every IK-generated joint trajectory must also fit
the joint envelope.

Planned streamed arm motion gives the STS3215 controller a deliberately more responsive
inner tracking profile (`Goal_Velocity=0`, `Acceleration=254`). The servo profile is not
a second authored trajectory and must not become a competing source of timing truth. Host
planning owns the intended trajectory; calibrated limits, following-error, effort/fault,
workspace, timing, and settle validation own acceptance.

The bounded agent broker does not define another motion envelope. The trusted broker process
selects an `SOARM101Config` envelope and injects those exact limits into every bounded
motion subprocess. Sandbox-side `robotctl` has no option or endpoint that can widen the
trusted-host envelope.

## Workstation and camera session boundary

Machine-local addressing is stored once in the workstation profile: follower/leader ports,
robot IDs/calibration-file references, and named camera profiles. Calibration contents remain
authoritative in calibration files; the workstation profile is only the local addressing map
consumed by GUI, CLI, and agents.

Machine-local workspace geometry is persisted separately under
`~/.config/soarm101/workspace/`. That artifact is tied to the active motor-calibration ID
and owns measured table/paper frame evidence; it is not duplicated into the workstation
profile or motor-calibration file.

The GUI may own multiple live camera sessions concurrently, but exactly one worker owns each
named physical device. Camera and Teleoperation views subscribe to those sessions rather than
opening their own device handles. CLI commands may open a camera for a short-lived capture
only when the GUI is not already using that physical device.

Camera workers publish preview frames into one synchronized latest-frame slot per named
camera. A GUI timer consumes those slots at display cadence, dropping superseded display
frames instead of queuing them as Qt image events. Snapshot requests remain on the capture
worker and save a freshly acquired observation.

The camera layer deliberately stops at raw observation: it does not identify objects, infer
task state, plan motion, or bypass motion safety. Agent reasoning remains above the same
constrained SDK primitives used manually.

## Bounded agent authority and CLI

The full CLI is an operator/developer surface. External reasoning agents use a smaller
deterministic facade:

```text
human interactive authorization
  -> time-limited robot+calibration authority lease
  -> soarm101 agent semantic capability
  -> existing SDK primitive
  -> existing deterministic safety guards
  -> hardware
```

The lease is machine-local runtime state under the user's local state directory, expires
automatically, and is invalid if the robot or calibration identity changes. Arming parks the
follower by latching/holding its current measured pose. Agent actions do not relax torque;
successful motion returns to holding. Human `soarm101 relax` remains outside the agent
surface and requires explicit interactive confirmation.

The bounded facade does not introduce a second motion or camera implementation. Named-pose
moves call the saved-pose SDK path, camera capture calls the same `CameraCapture` path, and
agent jog calls the normal guarded Cartesian jog. Only `agent_*` saved poses and the
configured logical `overhead`/`wrist` cameras are exposed. Agent jog is world/model-frame
translation only. A matching measured workspace calibration is consumed as *safety evidence*
to calculate physical current/target height and physical displacement; it does not replace
the existing model-space motion planner or promote workspace calibration to general execution
authority.

### Cartesian linear trajectory parameterization

`move_linear()` owns a Cartesian trajectory, not a sparse joint polyline. The host uses
symmetric half-cosine acceleration/deceleration with an optional constant-speed cruise,
then solves sequential IK directly at the actual command-rate samples. For position-only
paths, deterministic code may smooth the joint solution sequence only as a source of new
IK seeds, then re-solve each Cartesian sample at the unchanged hard tolerance and accept
the refined sequence only if joint jerk is lower. If forward continuation hits a numerical
IK pocket, an already validated endpoint solution may be used as a boundary-condition seed
to solve the same Cartesian samples backward; the fallback is accepted only when it
reconnects continuously to the measured start. Endpoint reachability does not prove that
the straight segment between endpoints is reachable, so supervised paper replay must
preflight every complete elevated segment read-only before powered traversal. The Cartesian
path remains authoritative throughout. Requested linear speed and acceleration remain ceilings of the profile;
position-only paths do not spend time rotating an unconstrained tool orientation. It must not introduce a second
piecewise-linear joint-space interpolation layer between sparse IK knots.

On calibrated Feetech hardware, planned Cartesian motion uses the same servo-side
tracking contract as live teleoperation: `speed_raw=0` gives the position loop full
tracking authority and `acceleration_raw=254` uses the validated responsive acceleration
profile. The host trajectory remains the single source of speed/acceleration shaping.
This is an execution requirement, not just a planning/documentation convention: Cartesian
`move_linear()` must not enable the per-sample synchronized servo-speed caps used by
optional joint-space moves. This avoids layering a second, quantized servo-speed trajectory
on top of 20/50 Hz host setpoints. The resulting joint samples remain subject to deterministic joint, step,
velocity, acceleration, workspace, following-error, effort, fault, and timing validation
before and during execution.

### Calibration-relative Sleep semantics

Resting postures remain deterministic SDK-owned primitives rather than machine-local taught
poses. `Sleep` is derived from the active executable calibration with shoulder pan and wrist
roll at midpoint, shoulder lift at lower, elbow flex at upper, and wrist flex at 75% of its
range. `sleep_up` preserves the historical wrist-at-lower-limit fold. After the arm fold,
the stock gripper closes to its calibration-derived target 1° inside the measured mechanical
stop. This action is not object-aware; a held pen caused the gripper servo to enter overload
protection during physical testing. Higher-level
scripts, agents, and broker clients call these semantic primitives; they do not own copied joint
coordinates.

