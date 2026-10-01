# Architecture

Dependency direction is strict:

```text
applications -> SOARM101 -> motion/kinematics/tools -> backend -> Feetech transport
applications -> CameraCapture -> OpenCV -> local USB/UVC camera
```

Rules:

- Basic local USB-camera discovery, named persisted capture profiles, live preview, and still-frame capture are owned here so CLI, GUI, and constrained agents share one deterministic observation surface.
- The arm has five pose joints. The stock motor-6 gripper is an `SO101Gripper` tool.
- Tools define motion-relevant TCP transforms. Tool/stage camera extrinsics, perception, tracking, OBS, and show-level capture orchestration remain higher-level concerns; this repository only owns the basic camera device session and raw frames.
- No LeRobot import exists in the runtime package.
- Hardware and simulation implement the same backend contract.
- Cartesian paths are validated before execution.
- GUI and CLI features call the same SDK operations and saved libraries; the GUI owns persistent hardware sessions rather than launching CLI subprocesses.
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
- A workspace calibration is measurement evidence until physical motion validation succeeds. The paper workflow's current supervised elevated traversal is a narrow validation exception that prioritizes **known-reachable endpoint construction**: the taught D→UP joint delta is applied to each taught A/B/C/D joint configuration and FK defines the Cartesian endpoints. Only D_UP has direct physical-height evidence, so affine-inverse heights at the other endpoints are diagnostic estimates rather than execution guarantees. The endpoints are connected with the ordinary `move_linear()` primitive. This does not authorize broader autonomous motion: the paper sequence is fixed, supervised, preflighted, and target-only coarse-workspace checked while preserving the deterministic joint/dynamic/hardware guard stack. Exact constant physical height across the paper requires additional elevated physical correspondences.
- Planned motion and live streaming share the core joint/rate/following-error/fault/effort safety stack, while live-stream workspace checks remain opt-in until the table frame and tool geometry are calibrated.


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

### Cartesian linear trajectory parameterization

`move_linear()` owns a Cartesian trajectory, not a sparse joint polyline. The host applies
its minimum-jerk timing law to Cartesian interpolation first and solves sequential IK at
the actual command-rate samples. It must not introduce a second piecewise-linear
joint-space interpolation layer between sparse IK knots, because those knot boundaries can
create derivative discontinuities that appear as physical corrections. The resulting
joint samples remain subject to deterministic joint, step, velocity, acceleration,
workspace, following-error, effort, fault, and timing validation before and during
execution.
