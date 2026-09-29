# Agent-facing SO-ARM101 CLI reference

This document describes the public SO-ARM101 command-line interface in a form suitable for
coding agents and other automation clients. It defines commands, units, outputs, and enforced
behavior only. It does not prescribe a task-solving strategy or action loop.

The authoritative runtime behavior remains the current CLI help and SDK source.

## Session selection

Commands that support either hardware or simulation accept:

```text
--port PORT
--robot-id ROBOT_ID
--calibration PATH
--simulation
```

Use `--simulation` instead of `--port` for software-only execution. Physical commands require
an active calibration unless the command explicitly documents otherwise.

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

## USB camera

List likely camera devices:

```bash
soarm101 camera list --json
```

Show persisted camera settings:

```bash
soarm101 camera show --json
```

Capture one still using persisted settings:

```bash
soarm101 camera capture --json
```

Capture with explicit temporary settings:

```bash
soarm101 camera capture \
  --device /dev/video0 \
  --width 1280 --height 720 --fps 30 --fourcc MJPG \
  --output observation.jpg \
  --json
```

Command-line capture does not rewrite persisted camera settings unless
`soarm101 camera configure` is used.

## Saved poses

```bash
soarm101 pose list --robot-id ROBOT_ID
soarm101 pose capture NAME --port PORT --robot-id ROBOT_ID
soarm101 pose go NAME --port PORT --robot-id ROBOT_ID --mode linear --yes
```

Saved physical poses carry calibration provenance. Replay fails closed when provenance does not
match the connected follower.

## Process semantics

- Commands requiring motion require `--yes`; otherwise they return exit code 2 without moving.
- Read-only commands do not enable torque.
- Normal connections do not rewrite motor configuration.
- Simulation uses the same high-level SDK operations without physical hardware.
- Software STOP/HOLD is not a certified emergency stop.
- SDK validation and safety failures are command failures; the CLI does not provide bypass flags
  for those guards.
