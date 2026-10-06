# CLI use

Technical reference for using the SO-ARM101 command-line interface.

For the complete command contract and structured-output details, see
`docs/agent-arm101-cli.md`. This file is a compact operational reference for the most
common CLI commands and coordinate conventions.

The current CLI help and SDK source are authoritative.

## Session selection

Most arm commands support either physical hardware or simulation.

Physical hardware:

```bash
soarm101 COMMAND --port /dev/ttyACM0 --robot-id so101
```

If the follower is saved in the workstation profile, `--port` may be omitted:

```bash
soarm101 workstation show --json
```

Simulation:

```bash
soarm101 COMMAND --simulation
```

Do not open the same physical serial port from the GUI and CLI simultaneously.

## Bounded agent control

For external reasoning agents, prefer the bounded facade over the unrestricted CLI:

```bash
# Human/operator step from an interactive terminal
soarm101 agent arm --minutes 60

# Agent-visible discovery/observation
soarm101 agent capabilities
soarm101 agent state
soarm101 agent poses
soarm101 agent cameras
soarm101 agent capture overhead
soarm101 agent capture wrist

# Bounded motion
soarm101 agent go-pose agent_start_overhead
soarm101 agent gripper open
soarm101 agent gripper close
soarm101 agent jog --x-mm 5 --y-mm 0 --z-mm 0
soarm101 agent sleep
soarm101 agent sleep-up
soarm101 agent stop
```

Agent motion authority is time-limited and bound to robot/calibration identity. Only
`agent_*` saved poses, logical `overhead`/`wrist` cameras, open/close gripper,
Sleep, the historical `sleep_up` override, STOP/HOLD, and translation-only jogs are exposed.

Agent jog uses the matching measured workspace transform to enforce requested physical
displacement limits of 50 mm when current physical height is above 100 mm and 10 mm when at
or below 100 mm. The planned target must remain at least 10 mm above the calibrated ground
plane, preserving margin for ordinary hardware settle/model error. These are command-space
bounds, not metrology guarantees. The normal SDK guards still apply. Successful actions
remain holding. Agents cannot relax torque.

```bash
# Human/operator only
soarm101 agent disarm
soarm101 relax
```

`relax` always requires explicit ENTER confirmation.

## Motion envelope

Inspect the effective host motion envelope without opening hardware:

```bash
soarm101 motion-envelope
soarm101 motion-envelope --json
```

The default absolute ceilings use a 100/1000 convention:

```text
joints        100 deg/s       1000 deg/s^2
TCP linear    100 mm/s        1000 mm/s^2
TCP angular   100 deg/s       1000 deg/s^2
```

Ordinary move defaults remain lower. These are maximum requested host dynamics, not servo
no-load ratings. Planned streamed arm motion uses the separate responsive Feetech tracking
profile `Goal_Velocity=0`, `Acceleration=254`.

For an explicit joint request in human units:

```bash
soarm101 move-joints J1 J2 J3 J4 J5 --degrees \
  --speed-deg-s 80 --acceleration-deg-s2 500 --yes
```

All session-capable commands also accept `--max-joint-...`, `--max-linear-...`, and
`--max-tool-angular-...` envelope overrides. When using `robotctl`, those values are
owned by the trusted broker and cannot be widened from the sandbox.

## Read-only commands

List candidate serial ports:

```bash
soarm101 ports
```

Discover SO-ARM101 devices:

```bash
soarm101 discover --json
```

Read the current joint positions and TCP pose:

```bash
soarm101 read --json
```

Read motor diagnostics:

```bash
soarm101 diagnose --json
```

Show the saved workstation configuration:

```bash
soarm101 workstation show --json
```

Show the saved mechanical calibration ranges, nominal model limits, effective runtime
limits, and coarse Cartesian envelope without opening hardware:

```bash
soarm101 limits --robot-id so101 --json
```

Use `--calibration PATH` to inspect a specific calibration file.

`limits` reports three ranges for each pose joint:

```text
calibrated = measured mechanical stop-to-stop range from the saved calibration
model      = nominal SO-101 kinematic-model planning range
effective  = executable range for this arm
```

The URDF/model range is the generic fallback and reference. Once a real arm has a saved
mechanical-stop calibration, executable pose-joint travel follows that arm-specific
calibration while staying 1° inside each measured stop by default. If a measured range is
narrower than the URDF range, calibration still remains the physical authority. The stop
itself is never a normal command target.

The coarse Cartesian `maximum_tcp_reach` is an outer radial envelope. It does not mean
that every XYZ point inside that radius is reachable.

These commands do not command arm motion.

## TCP coordinates

`soarm101 read --json` reports:

```json
{
  "joint_positions_rad": {
    "shoulder_pan": 0.0,
    "shoulder_lift": 0.0,
    "elbow_flex": 0.0,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0
  },
  "tcp_xyz_mm": [X, Y, Z],
  "tcp_rpy_deg": [ROLL, PITCH, YAW]
}
```

`tcp_xyz_mm` is the active tool center point (TCP) in the SDK's
`soarm101/base` model frame.

The three coordinates are orthogonal model axes:

```text
X = model X axis
Y = model Y axis
Z = model Z axis
```

Positive and negative values indicate opposite directions along each axis.

The CLI does not define universal human labels such as:

```text
+X = right
+Y = forward
+Z = physical up
```

Those labels depend on how the arm and workspace are physically arranged.

In particular, model/base XYZ and a calibrated physical workspace are distinct coordinate
systems. The normal Cartesian CLI commands use model/base coordinates.

## Calibrated Sleep posture

After mechanical-stop calibration, Sleep is derived from that follower's own executable
joint limits rather than hard-coded angles:

```text
shoulder_pan   midpoint
shoulder_lift  lower limit
elbow_flex     upper limit
wrist_flex     lower + 0.75 * (upper - lower)
wrist_roll     midpoint
```

Equivalently, the default wrist target is `upper - 0.25 * (upper - lower)`. The
historical fully folded wrist-up posture remains available as `sleep_up`, with
`wrist_flex` at the lower executable limit. The lower/upper limits already include the
configured 1° inset from measured stops.

```bash
soarm101 sleep --speed-deg-s 8 --acceleration-deg-s2 25 --yes
soarm101 sleep-up --speed-deg-s 8 --acceleration-deg-s2 25 --yes
```

`soarm101 limits --robot-id so101 --json` reports both `sleep_pose_deg` and
`sleep_up_pose_deg` without moving hardware. These are normal guarded joint-space moves
and are never commanded automatically on connect or torque enable. After either operator
CLI move finishes, the CLI holds the arm and waits for ENTER before disabling torque.

## Relative Cartesian jog

Use `jog` for a relative Cartesian displacement.

Example: move +2 mm along model X:

```bash
soarm101 jog \
  --frame world \
  --x-mm 2 \
  --y-mm 0 \
  --z-mm 0 \
  --orientation-mode position_only \
  --speed-mm-s 10 \
  --acceleration-mm-s2 40 \
  --json \
  --yes
```

Negative values move in the opposite direction:

```bash
soarm101 jog \
  --frame world \
  --x-mm -2 \
  --y-mm 0 \
  --z-mm 0 \
  --orientation-mode position_only \
  --speed-mm-s 10 \
  --acceleration-mm-s2 40 \
  --json \
  --yes
```

### Jog frames

```text
--frame world
```

interprets XYZ deltas in the fixed SDK base/model frame.

```text
--frame tool
```

interprets XYZ deltas in the current TCP frame. Tool-frame axes rotate with the tool
orientation.

## Absolute Cartesian move

Use `move-linear` for an absolute Cartesian TCP target:

```bash
soarm101 move-linear \
  --x-mm X \
  --y-mm Y \
  --z-mm Z \
  --orientation-mode position_only \
  --speed-mm-s 10 \
  --acceleration-mm-s2 40 \
  --json \
  --yes
```

The target XYZ values are absolute model/base-frame coordinates in millimeters.

`move-linear` requests a straight Cartesian TCP path from the current pose to the target.
The SDK plans and validates the complete path before executing it.

Endpoint reachability does not guarantee that the straight path between the current pose
and endpoint is reachable.

## Read-only inverse kinematics

Use `ik` to solve for a Cartesian endpoint without commanding motion:

```bash
soarm101 ik \
  --x-mm X \
  --y-mm Y \
  --z-mm Z \
  --orientation-mode position_only \
  --json
```

Simulation:

```bash
soarm101 ik \
  --simulation \
  --x-mm X \
  --y-mm Y \
  --z-mm Z \
  --orientation-mode position_only \
  --json
```

Typical output includes:

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

## Orientation modes

Cartesian commands support these orientation modes:

```text
position_only
compatible
exact
```

`position_only`
: Constrain XYZ and allow TCP orientation to vary.

`compatible`
: Constrain position while preferring an orientation compatible with the five-axis arm.

`exact`
: Require the requested XYZ and roll/pitch/yaw pose to be reachable.

Rotation arguments, where supported, are:

```text
--roll-deg
--pitch-deg
--yaw-deg
```

## Absolute joint move

Move the five pose joints directly:

```bash
soarm101 move-joints \
  J1 J2 J3 J4 J5 \
  --degrees \
  --json \
  --yes
```

Without `--degrees`, joint values are radians.

Joint order is:

```text
shoulder_pan
shoulder_lift
elbow_flex
wrist_flex
wrist_roll
```

A joint-space move does not imply a straight Cartesian TCP path.

## Gripper

Open:

```bash
soarm101 gripper open --json --yes
```

Close:

```bash
soarm101 gripper close --json --yes
```

Normalized numeric position:

```bash
soarm101 gripper 0.5 --json --yes
```

Numeric range:

```text
0.0 = closed
1.0 = open
```

## Saved poses

List:

```bash
soarm101 pose list --robot-id so101
```

Capture the current pose:

```bash
soarm101 pose capture NAME --robot-id so101
```

Replay a saved pose:

```bash
soarm101 pose go NAME \
  --robot-id so101 \
  --mode linear \
  --yes
```

Saved physical poses are tied to calibration provenance. A successful physical
`pose go` leaves the follower holding the reached pose even after the CLI disconnects.
It does not relax automatically.

When joint/angular replay starts from a measured pose that is already inside the coarse
self-clearance envelope, the SDK may permit a controlled exit. All non-self-clearance
workspace checks must pass at every sample, minimum self-clearance may not decrease while
inside the envelope, and the path must reach the normal clearance threshold. Ordinary full
workspace checking resumes from that point.

To release torque manually:

```bash
soarm101 relax --robot-id so101
```

The relax command always requires an explicit ENTER confirmation; there is no non-interactive
confirmation bypass.

## Cameras

List configured and detected cameras:

```bash
soarm101 camera list --json
```

Show camera configuration:

```bash
soarm101 camera show --json
```

Capture one configured camera:

```bash
soarm101 camera capture --name overhead --json
```

Capture all configured cameras:

```bash
soarm101 camera capture --all --json
```

## JSON motion results

Commands documented with `--json` return machine-readable output.

Motion commands return a structure like:

```json
{
  "accepted": true,
  "completed": true,
  "message": null,
  "final_positions": {}
}
```

A rejected command returns a non-zero process exit code.

## Motion confirmation

Physical motion commands require explicit confirmation.

For non-interactive use:

```text
--yes
```

must be supplied.

Without `--yes`, motion commands refuse to execute.

## Coordinate-system distinction

Three coordinate concepts may appear in this repository:

```text
joint coordinates
    servo/joint angles

model/base Cartesian coordinates
    TCP X/Y/Z used by read, ik, jog --frame world, and move-linear

calibrated physical workspace coordinates
    a separately measured mapping between a physical work surface and model space
```

They are not interchangeable.

The ordinary Cartesian CLI uses model/base coordinates.

Do not infer a physical table direction solely from the name of a model axis unless that
relationship has been separately measured for the current hardware setup.

## Safety and validation behavior

The CLI calls the same guarded SDK operations used by the GUI and Python API.

Depending on the command, the SDK enforces:

- motor calibration and calibration provenance;
- effective joint limits;
- Cartesian IK tolerance and continuity;
- maximum command step;
- joint speed and acceleration limits;
- workspace checks;
- following-error checks;
- effort/current safety checks;
- motor fault checks;
- communication checks;
- command timing/deadline checks; and
- measured completion/settling.

The CLI does not provide a flag to bypass those guards.

Software STOP/HOLD is not a physical emergency stop.

## Full CLI reference

For additional commands, exact option definitions, structured output schemas, camera
configuration, workstation configuration, and process semantics, see:

```text
docs/agent-arm101-cli.md
```

Use the installed command help as the final authority for the current checkout:

```bash
soarm101 --help
soarm101 COMMAND --help
```
