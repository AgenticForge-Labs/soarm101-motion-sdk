# Agent-facing SO-ARM101 CLI reference

This document describes the public SO-ARM101 command-line interface in a form suitable for
coding agents and other automation clients. It defines commands, units, outputs, and enforced
behavior only. It does not prescribe a task-solving strategy or action loop.

The authoritative runtime behavior remains the current CLI help and SDK source.

The machine-local follower/leader addressing and named cameras are exposed through:

```bash
soarm101 workstation show --json
```

This reads `~/.config/soarm101/workstation.json` and does not open hardware.

## Bounded external-agent surface

External reasoning agents should use the smaller `soarm101 agent ...` facade rather than
the full operator/developer CLI. All agent commands emit JSON.

A human explicitly starts motion authority from an interactive terminal:

```bash
soarm101 agent arm --minutes 60
```

Arming parks/holds the follower and writes a time-limited authority lease under the user's
local state directory. The lease is bound to robot ID and calibration ID. Arming is rejected
when stdin is non-interactive, and motion fails closed if the lease is missing, expired, or
does not match the connected robot/calibration.

Read-only discovery does not require authority:

```bash
soarm101 agent capabilities
soarm101 agent state
soarm101 agent poses
soarm101 agent cameras
soarm101 agent capture overhead
soarm101 agent capture wrist
```

When a saved paper/workspace calibration is available, `agent capabilities` also reports
`world_directions`. The paper calibration defines physical +X as A->B, physical +Y as
A->D/B->C, and physical +Z as the manually measured D->UP direction. The CLI labels those
as right/left, forward/back, and up/down respectively and reports the corresponding
model/world XYZ delta per physical millimeter. Agents can multiply that vector by the
requested physical distance and pass the result to the existing `agent jog` command.
No separate semantic-motion primitive is introduced.

Only saved poses beginning with `agent_` are exposed:

```bash
soarm101 agent go-pose agent_start_overhead
```

The bounded gripper surface is:

```bash
soarm101 agent gripper open
soarm101 agent gripper close
```

These targets stay one configured calibrated angular margin inside the corresponding
mechanical endpoint on physical hardware.

The bounded agent also exposes one-joint relative adjustments:

```bash
soarm101 agent joint shoulder_pan --delta-deg 20
```

Only one named pose joint changes per command, the absolute delta is capped at 30 degrees,
normal calibrated joint/workspace/path checks remain active, and the follower remains held
after completion.

The bounded Cartesian surface is translation-only and may use either the fixed SDK
base/model frame or the current gripper/TCP frame:

```bash
soarm101 agent jog --frame world --x-mm 5 --y-mm 0 --z-mm 0
soarm101 agent jog --frame tool --x-mm 0 --y-mm 0 --z-mm 5
```

Tool-frame XYZ follows the current TCP axes and therefore rotates with the gripper.

The command requires a matching saved workspace calibration and applies an additional
physical-space policy before the normal SDK jog:

- current physical height > 100 mm: maximum physical displacement 50 mm;
- current physical height <= 100 mm: maximum physical displacement 10 mm;
- target physical height below 0 mm/calibrated ground plane: rejected.

The displacement limit is the norm of the inverse-mapped physical displacement, not an
independent per-axis allowance. The workspace mapping is used only for this safety
measurement; actual motion still executes through the normal model-frame guarded jog.

Sleep and STOP/HOLD are:

```bash
soarm101 agent sleep
soarm101 agent stop
```

Successful agent motion remains torque-held at the reached pose. `agent stop` is available
even without active motion authority. Relax is intentionally not exposed to the agent.
A human releases torque separately with `soarm101 relax`, which requires explicit ENTER
confirmation.

A human may remove future agent motion authority without changing the current hold:

```bash
soarm101 agent disarm
```

The optional task-specific robot/camera skill under `agent-as-code/` is outside this
technical contract.

## Session selection

Commands that support either hardware or simulation accept:

```text
--port PORT
--robot-id ROBOT_ID
--calibration PATH
--simulation
```

Use `--simulation` instead of a physical port for software-only execution. For physical
session commands, omitting `--port` uses the saved follower entry from
`~/.config/soarm101/workstation.json`. An explicit `--port`, `--robot-id`, or
`--calibration` overrides the corresponding saved value. Physical commands require an active
calibration unless the command explicitly documents otherwise.

The GUI and CLI must not open the same follower serial port at the same time. A GUI-owned USB
camera stream likewise cannot be opened concurrently by a separate CLI capture process.

## Machine-readable output

Commands documented with `--json` emit JSON to stdout on success. Error messages and rejected
commands use a non-zero process exit code.

Motion-result JSON has this shape:

```json
{
  "accepted": true,
  "completed": true,
  "message": null,
  "final_positions": {
    "shoulder_pan": 0.0
  }
}
```

`final_positions` contains measured final positions returned by the underlying guarded SDK
operation. Arm joints are radians. The stock gripper, when present, is normalized to `0..1`.

## Discover serial devices

```bash
soarm101 ports
soarm101 discover --json
```

`discover` is read-only and does not enable torque.

## Read current joints and TCP

```bash
soarm101 read --port PORT --robot-id ROBOT_ID --json
```

or in simulation:

```bash
soarm101 read --simulation --json
```

JSON fields:

```json
{
  "joint_positions_rad": {
    "shoulder_pan": 0.0,
    "shoulder_lift": 0.0,
    "elbow_flex": 0.0,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0
  },
  "tcp_xyz_mm": [0.0, 0.0, 0.0],
  "tcp_rpy_deg": [0.0, 0.0, 0.0]
}
```

The TCP is the SDK's active stock-gripper TCP in the `soarm101/base` frame.

## Saved calibration and workspace limits

```bash
soarm101 limits --robot-id ROBOT_ID --json
```

This command reads the saved calibration file only; it does not open the serial port or
enable torque. `--calibration PATH` selects a specific calibration file.

For each pose joint it reports:

- `calibrated_rad` / `calibrated_deg`: measured mechanical-stop range;
- `model_rad` / `model_deg`: nominal kinematic-model planning range; and
- `effective_rad` / `effective_deg`: executable range for the calibrated arm.

The URDF/model limits are the generic fallback/reference. For a calibrated real arm,
normal executable pose-joint authority follows the saved mechanical-stop calibration with
a 1° inset from each measured stop by default. `calibrated_joint_stop_margin_deg`
reports that policy. The same output includes `sleep_pose_rad` and `sleep_pose_deg`,
derived from those executable limits, plus `sleep_gripper` and
`calibrated_gripper_stop_margin_deg`. The gripper Sleep target is the calibrated closed
mechanical stop inset 1° toward open by default and is reported in both normalized and raw
encoder coordinates. Calibration remains the physical authority if a measured range is
narrower than the model range.

It also reports the configured coarse model-space Cartesian envelope, including maximum
TCP reach, minimum model Z, base keep-out dimensions, and minimum self-clearance. Maximum
TCP reach is an outer radial guard, not a guarantee that every XYZ point inside it is
kinematically reachable.

## Motor diagnostics

```bash
soarm101 diagnose --port PORT --robot-id ROBOT_ID --json
```

This reads motor model, raw position, temperature, voltage, current, moving state, status, and
communication errors without enabling torque.

## Solve inverse kinematics without motion

```bash
soarm101 ik \
  --port PORT \
  --robot-id ROBOT_ID \
  --x-mm X --y-mm Y --z-mm Z \
  --orientation-mode position_only \
  --json
```

Simulation is also supported:

```bash
soarm101 ik --simulation --x-mm X --y-mm Y --z-mm Z \
  --orientation-mode position_only --json
```

The command connects only to read the current arm configuration used as the IK seed. It does not
enable torque or command motion.

JSON fields include:

```json
{
  "success": true,
  "joints_rad": {},
  "joints_deg": {},
  "position_error_mm": 0.0,
  "orientation_error_deg": 0.0,
  "iterations": 0,
  "message": ""
}
```

Orientation modes are:

- `position_only`: constrain XYZ and allow orientation to float.
- `compatible`: constrain position and prefer an orientation compatible with the five-axis arm.
- `exact`: require the requested XYZ/RPY pose to lie on the reachable five-axis manifold.

A failed solve exits non-zero.

## Calibrated Sleep posture

```bash
soarm101 sleep --speed-deg-s 8 --acceleration-deg-s2 25 --yes
```

Sleep is computed from the active follower's executable joint limits: shoulder pan
midpoint, shoulder lift lower limit, elbow flex upper limit, wrist flex lower limit, and
wrist roll midpoint. On a calibrated arm the endpoint limits are already inset 1° from the
measured mechanical stops. After the arm reaches that fold, the stock gripper closes to a
target 1° inside its calibrated closed mechanical stop by default. The target is derived
from the saved gripper encoder range and normalized so calibration handles either motor
drive direction. Sleep is never triggered automatically by connection or torque enable. After the commanded
Sleep move completes, the CLI keeps torque enabled and waits for the operator to press ENTER
before it relaxes the arm.

## Relative Cartesian linear jog

```bash
soarm101 jog \
  --port PORT \
  --robot-id ROBOT_ID \
  --frame world \
  --x-mm DX --y-mm DY --z-mm DZ \
  --orientation-mode compatible \
  --speed-mm-s 10 \
  --acceleration-mm-s2 40 \
  --json \
  --yes
```

`--frame world` interprets the delta in the SDK base frame. `--frame tool` interprets it in
the current TCP frame.

Rotation deltas may be supplied with `--roll-deg`, `--pitch-deg`, and `--yaw-deg`.
`position_only` cannot be combined with a non-zero rotation jog.

The SDK constructs and validates the complete Cartesian path before motor commands.

## Absolute Cartesian linear move

```bash
soarm101 move-linear \
  --port PORT \
  --robot-id ROBOT_ID \
  --x-mm X --y-mm Y --z-mm Z \
  --orientation-mode position_only \
  --speed-mm-s 10 \
  --acceleration-mm-s2 40 \
  --json \
  --yes
```

Optional orientation arguments are `--roll-deg`, `--pitch-deg`, and `--yaw-deg`.

`move-linear` plans a Cartesian path and solves IK sequentially along that path. The configured
joint, workspace, speed, acceleration, following-error, motor-fault, effort, calibration, and
completion checks remain authoritative.

## Absolute joint move

```bash
soarm101 move-joints \
  --port PORT \
  --robot-id ROBOT_ID \
  J1 J2 J3 J4 J5 \
  --degrees \
  --json \
  --yes
```

Without `--degrees`, the five positional arguments are radians.

## Stock gripper

```bash
soarm101 gripper --port PORT --robot-id ROBOT_ID open --json --yes
soarm101 gripper --port PORT --robot-id ROBOT_ID close --json --yes
soarm101 gripper --port PORT --robot-id ROBOT_ID 0.5 --json --yes
```

The normalized numeric range is `0..1`.

## USB cameras

List both discovered devices and persisted camera names:

```bash
soarm101 camera list --json
```

Show the complete registry or one named camera:

```bash
soarm101 camera show --json
soarm101 camera show --name overhead --json
```

Capture a configured camera by logical name:

```bash
soarm101 camera capture --name overhead --json
soarm101 camera capture --name wrist --json
```

Capture all configured cameras sequentially:

```bash
soarm101 camera capture --all --json
```

A successful single-camera capture includes the logical `name`, saved `path`, timestamp,
dimensions, and device. `--all` returns a `captures` array.

Camera profiles can be created or changed explicitly:

```bash
soarm101 camera configure \
  --name overhead \
  --device /dev/v4l/by-id/CAMERA-video-index0 \
  --width 1280 --height 720 --fps 30 --fourcc MJPG \
  --json
```

On Linux, discovery prefers stable `/dev/v4l/by-id/*-video-index0` paths when available.
A physical device may belong to only one saved camera profile. CLI capture must not open a
camera device currently owned by the GUI.

## Saved poses

```bash
soarm101 pose list --robot-id ROBOT_ID
soarm101 pose capture NAME --port PORT --robot-id ROBOT_ID
soarm101 pose go NAME --port PORT --robot-id ROBOT_ID --mode linear --yes
```

Saved physical poses carry calibration provenance. Replay fails closed when provenance does not
match the connected follower. Joint/angular saved-pose replay can also leave a measured starting
configuration that is already inside the coarse centerline self-clearance envelope. This is not
a bypass: every floor/reach/base guard must pass for every sample, minimum self-clearance may not
decrease while the path remains inside the envelope, the path must eventually reach the configured
clearance threshold, and ordinary full workspace validation becomes authoritative from that point
onward. This permits a real folded/parked pose to unfold without treating sample 0 as a newly
commanded collision.

A successful physical `pose go` intentionally leaves follower torque enabled after the CLI
disconnects, so the robot remains holding the reached pose. It does not relax automatically.
Use `soarm101 relax` when a human is ready to release the arm; that command always waits for
an explicit ENTER confirmation before disabling torque.

## Process semantics

- Commands requiring motion require `--yes`; otherwise they return exit code 2 without moving.
- Read-only commands do not enable torque.
- Normal connections do not rewrite motor configuration.
- Simulation uses the same high-level SDK operations without physical hardware.
- Software STOP/HOLD is not a certified emergency stop.
- SDK validation and safety failures are command failures; the CLI does not provide bypass flags
  for those guards.
