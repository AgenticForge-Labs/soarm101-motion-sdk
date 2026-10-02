# Drive me: agent-facing SO-ARM101 operating guide

This file is the practical operating guide for an agent that needs to inspect and move an
SO-ARM101 through the supported CLI.

It is intentionally simpler than the full SDK documentation. The authoritative command
contract remains `docs/agent-arm101-cli.md`, and the SDK remains responsible for
calibration, limits, path validation, following error, effort/fault checks, timing, and
safe motion execution.

The basic rule is:

```text
agent decides intent
    ↓
CLI expresses a constrained request
    ↓
SO-ARM101 Motion SDK validates and executes it
    ↓
hardware
```

Do not write directly to servo registers to accomplish ordinary tasks.

## The most important thing about XYZ

The CLI reports the active tool center point (TCP) as:

```text
X, Y, Z in millimeters
```

These coordinates are in the SDK's `soarm101/base` **model frame**.

Think of them abstractly as three perpendicular directions:

```text
X = horizontal model axis A
Y = horizontal model axis B, 90 degrees from X
Z = model vertical axis

+X / -X are opposite directions
+Y / -Y are opposite directions
+Z / -Z are opposite directions
```

Human words such as **left/right**, **forward/back**, and **up/down** are semantic labels
that must be mapped onto those model axes for the actual bench setup.

Do **not** assume, without validation, that:

```text
+X = right
+Y = forward
+Z = physical up
```

or any other particular assignment.

This matters on the current hardware: physical testing has already shown that a model
`+Z` request cannot automatically be interpreted as physical "up" relative to the table.

A useful one-time physical direction record is therefore:

```text
THIS BENCH

+X = ____________________
-X = ____________________
+Y = ____________________
-Y = ____________________
+Z = ____________________
-Z = ____________________
```

Fill those semantic directions only after very small supervised motions or another
independent physical measurement confirms them.

### Base/world frame versus tool frame

For an agent learning the workspace, prefer:

```bash
--frame world
```

A world-frame jog means the XYZ delta is expressed in the fixed SDK base/model frame.

A tool-frame jog:

```bash
--frame tool
```

rotates XYZ with the current tool orientation. Tool-frame `+X`, for example, is not a
fixed direction in the room as the wrist changes orientation.

Start with world-frame motion unless the task specifically benefits from tool-relative
movement.

## Model XYZ is not calibrated paper/workspace XYZ

The normal CLI Cartesian commands use the SDK **model/base frame**.

Commands such as:

```text
read
ik
jog --frame world
move-linear
```

do not mean "millimeters above the physical table."

The separate paper/workspace calibration records a measured mapping between a physical
work surface and the model frame. That mapping is useful evidence, but it is not currently
a general-purpose replacement coordinate system for arbitrary CLI Cartesian motion.

Therefore an agent must keep these concepts separate:

```text
model/base XYZ       = the coordinates used by the normal motion CLI
physical workspace   = real table/bench directions and distances
workspace calibration = measured mapping between the two
```

## Start every session by observing

Show the machine-local follower and camera configuration:

```bash
soarm101 workstation show --json
```

Read the current joints and TCP without moving:

```bash
soarm101 read --json
```

Typical output includes:

```json
{
  "joint_positions_rad": {
    "shoulder_pan": 0.0,
    "shoulder_lift": 0.0,
    "elbow_flex": 0.0,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0
  },
  "tcp_xyz_mm": [123.4, 56.7, 89.0],
  "tcp_rpy_deg": [0.0, 0.0, 0.0]
}
```

The three `tcp_xyz_mm` values are the current absolute model-frame X, Y, and Z.

Read motor diagnostics without motion:

```bash
soarm101 diagnose --json
```

If cameras are configured, get a fresh observation:

```bash
soarm101 camera capture --name overhead --json
```

or:

```bash
soarm101 camera capture --all --json
```

## Ask whether a point is reachable without moving

Before an unfamiliar absolute Cartesian move, use inverse kinematics:

```bash
soarm101 ik \
  --x-mm X \
  --y-mm Y \
  --z-mm Z \
  --orientation-mode position_only \
  --json
```

This is read-only. It does not enable torque or command motion.

For many agent tasks, `position_only` is the simplest mode because it constrains XYZ
while allowing the five-axis arm to choose a reachable wrist orientation.

A successful endpoint IK solution means the endpoint is reachable. It does **not** prove
that every point on a straight Cartesian path to that endpoint is reachable.
`move-linear` performs the full path planning and can still reject such a move.

## Small relative motion: use jog

For exploration and local corrections, prefer a small relative jog.

Move +2 mm along model X:

```bash
soarm101 jog \
  --frame world \
  --x-mm 2 --y-mm 0 --z-mm 0 \
  --orientation-mode position_only \
  --speed-mm-s 10 \
  --acceleration-mm-s2 40 \
  --json --yes
```

Move -2 mm along model X:

```bash
soarm101 jog \
  --frame world \
  --x-mm -2 --y-mm 0 --z-mm 0 \
  --orientation-mode position_only \
  --speed-mm-s 10 \
  --acceleration-mm-s2 40 \
  --json --yes
```

The same pattern applies to Y and Z.

For an unfamiliar physical setup, use small motions to establish the semantic axis map
rather than assuming which sign means left, right, forward, back, up, or down.

After every exploratory move, read state again:

```bash
soarm101 read --json
```

and, when vision matters, capture a fresh camera frame.

## Absolute Cartesian motion: use move-linear

Once an absolute model-frame target is known:

```bash
soarm101 move-linear \
  --x-mm X \
  --y-mm Y \
  --z-mm Z \
  --orientation-mode position_only \
  --speed-mm-s 10 \
  --acceleration-mm-s2 40 \
  --json --yes
```

This requests a **straight Cartesian TCP path** from the current pose to the requested
model-frame XYZ.

That is a stronger constraint than merely asking the arm to reach the endpoint. A target
can pass `ik` while `move-linear` correctly rejects the intervening straight line.

Do not respond to such a rejection by disabling guards or increasing tolerances. Choose
another validated route, smaller motion, or a known safe capability.

## Joint motion

Direct joint motion is available:

```bash
soarm101 move-joints J1 J2 J3 J4 J5 --degrees --json --yes
```

but agents should normally reason in named poses or Cartesian intent unless the task is
specifically about joints, calibration, recovery, or robot development.

Joint-space movement and Cartesian movement are different:

```text
move-joints:
    move through joint configuration space
    TCP may follow a curved 3-D path

move-linear:
    keep the TCP on the requested straight Cartesian path
    planner solves the required joint motion along that line
```

The fact that the arm can reach two endpoints with joint motion does not guarantee that
the straight Cartesian line between them is reachable.

## Saved poses

List known poses:

```bash
soarm101 pose list --robot-id so101
```

Capture the current pose:

```bash
soarm101 pose capture NAME --robot-id so101
```

Move to a saved pose:

```bash
soarm101 pose go NAME --robot-id so101 --mode linear --yes
```

Saved physical poses carry calibration provenance. Do not bypass a provenance failure after
recalibration.

For recurring agent tasks, meaningful validated names such as:

```text
rest
home
above_pick
pick
above_drop
drop
camera_view
```

are often safer and easier to reason about than repeatedly inventing raw coordinates.

## Gripper

Open:

```bash
soarm101 gripper open --json --yes
```

Close:

```bash
soarm101 gripper close --json --yes
```

Set an intermediate normalized position:

```bash
soarm101 gripper 0.5 --json --yes
```

The numeric range is `0..1`.

## A simple agent loop

A good minimal control loop is:

```text
1. OBSERVE
   read current joints/TCP
   capture camera frame if useful

2. INTERPRET
   decide the desired small change or validated target

3. PREFLIGHT
   use IK for unfamiliar absolute Cartesian targets
   use known poses/capabilities when available

4. ACT
   issue one bounded CLI motion request

5. VERIFY
   inspect the JSON result
   read the new TCP
   capture a fresh image if the task depends on vision

6. CONTINUE OR STOP
```

Do not make a long chain of blind Cartesian moves when fresh state can be checked between
actions.

## How to establish human-readable XYZ semantics

For a new setup, the goal is to determine statements such as:

```text
+X moves the TCP toward the robot's right
-X moves the TCP toward the robot's left
+Y moves away from the base
-Y moves toward the base
+Z moves physically upward
-Z moves physically downward
```

Those are examples of the **kind** of semantic map an agent wants, not claims about the
current arm.

Establish the actual map with tiny supervised world-frame jogs:

```text
read
small +X jog
observe
return with -X

small +Y jog
observe
return with -Y

small +Z jog
observe
return with -Z
```

Record the observed physical meanings in the six-line bench map near the top of this file.

Once that map is physically validated, higher-level prompts can use ordinary language such
as "move left 5 mm" by deterministically translating "left" into the corresponding signed
model-axis delta.

That translation should be configuration/data, not something the language model guesses on
every move.

## Orientation

The CLI also represents TCP orientation as roll, pitch, and yaw.

For early agent control, translation with:

```bash
--orientation-mode position_only
```

is usually the easiest way to learn the XYZ workspace because it avoids unnecessarily
constraining the five-axis arm's wrist orientation.

Use `compatible` when orientation matters but some five-axis flexibility is acceptable.
Use `exact` only when the full requested orientation is genuinely required and reachable.

## What a successful command means

Motion JSON reports fields such as:

```json
{
  "accepted": true,
  "completed": true,
  "message": null,
  "final_positions": {}
}
```

A successful command means the SDK accepted and completed that guarded request.

It does not mean:

- every neighboring Cartesian point is reachable;
- the model frame has been semantically mapped to the physical bench;
- model Z is known physical up;
- simulation proves hardware safety; or
- arbitrary autonomous Cartesian operation has been physically validated.

## Failure behavior

If a command fails:

1. stop the action sequence;
2. preserve the error text/JSON;
3. read the current state again;
4. do not automatically loosen limits or tolerances;
5. do not retry with a larger move;
6. choose a smaller or differently constrained request only when the failure is understood.

Software STOP/HOLD is not a physical emergency stop. For supervised hardware development,
physical power must remain reachable.

## Recommended agent capability hierarchy

Prefer the highest-level already-validated capability that solves the task:

```text
validated sequence / named pose
        ↓
small Cartesian jog
        ↓
preflighted absolute Cartesian move
        ↓
joint-space move when the task specifically requires it
        ↓
raw motor access only for explicit low-level hardware development
```

The agent chooses **what** should happen. The deterministic SDK remains responsible for
whether the requested motion is valid and how it is executed.

## Current autonomy status

The CLI and SDK are suitable for agent integration now: they provide structured readout,
read-only IK, guarded relative/absolute motion, joint motion, gripper control, saved poses,
camera capture, and deterministic failures.

However, the physical meaning of arbitrary model-frame Cartesian directions must be
validated on the actual bench before an unsupervised agent is allowed to interpret model
XYZ as physical left/right/forward/back/up/down.

The recent paper experiments should therefore be understood as coordinate-system and
Cartesian-path validation work, not as a prerequisite for giving an agent access to the
CLI's already-constrained capabilities.
