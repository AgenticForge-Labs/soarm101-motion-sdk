---
name: soarm101-robot-camera
description: Operate the bounded SO-ARM101 robot and overhead/wrist cameras for object-to-container manipulation tasks.
---

# Bounded SO-ARM101 robot + camera skill

Use this skill only for an already configured SO-ARM101 workstation where a human has
explicitly armed the bounded agent session.

## Known observation tools

The agent may assume these logical camera names from the start:

- `overhead`: primary workspace/status camera.
- `wrist`: robot-mounted close view for fine manipulation.

A camera name is usable only when it is currently configured. Check with:

```bash
soarm101 agent cameras
```

Capture a fresh image with:

```bash
soarm101 agent capture overhead
soarm101 agent capture wrist
```

The command returns JSON containing the saved image path. Inspect that image with the
agent's normal image-viewing capability. Do not infer scene state from an old image when a
fresh observation is available.

## Motion authority

A human starts motion authority before the agent run:

```bash
soarm101 agent arm
```

The agent must never attempt to arm, disarm, or relax the robot itself. If motion authority
is absent or expired, report that human re-arming is required.

Inspect the bounded action surface with:

```bash
soarm101 agent capabilities
soarm101 agent state
soarm101 agent poses
```

Only use the `soarm101 agent ...` robot-control surface. Do not import the Python SDK,
call the unrestricted `soarm101 move-*` commands, change calibration/configuration, or
write servo registers. The agent runner should enforce this command boundary when possible;
the human-arming lease is an operational gate and is not a substitute for shell sandboxing
against an agent with unrestricted same-user filesystem/process access.

## Allowed robot actions

Move to an existing agent-approved pose:

```bash
soarm101 agent go-pose agent_start_overhead
```

Only saved poses whose names begin with `agent_` are available through this interface.
Useful installed viewpoints may include:

- `agent_start_overhead`
- `agent_start_overhead_left`
- `agent_start_overhead_right`

Discover the actual list rather than assuming every example exists.

Adjust exactly one named arm joint by a relative angle:

```bash
soarm101 agent joint shoulder_pan --delta-deg 20
soarm101 agent joint shoulder_lift --delta-deg -5
soarm101 agent joint elbow_flex --delta-deg 5
soarm101 agent joint wrist_flex --delta-deg -5
soarm101 agent joint wrist_roll --delta-deg 10
```

Use this when the intent is explicitly about a joint angle or articulation. The bounded
agent command changes only one joint per action, enforces a per-command angle limit, uses
the normal joint/workspace/path safety checks, and remains torque-held afterward. Prefer
small adjustments and re-observe after consequential changes.

For `shoulder_pan`, the current SO-ARM101 kinematic model establishes that positive
`--delta-deg` rotates clockwise when viewed from directly above the base; negative rotates
counterclockwise. For other joints, use the signed joint-coordinate convention rather than
inventing human directional names unless they have been explicitly established.

Open or close the gripper:

```bash
soarm101 agent gripper open
soarm101 agent gripper close
```

Make a bounded Cartesian translational adjustment in either the calibrated world/model
frame or the current gripper/TCP frame:

```bash
# fixed world/model coordinates
soarm101 agent jog --frame world --x-mm 5 --y-mm 0 --z-mm 0

# current gripper/TCP coordinates
soarm101 agent jog --frame tool --x-mm 0 --y-mm 0 --z-mm 5
```

Use `--frame world` for table/workspace directions such as calibrated left/right/up/down.
Use `--frame tool` when the requested motion is from the gripper's own current perspective
(for example, a small approach/retract/lateral correction aligned with the tool). Tool-frame
X/Y/Z are the current TCP axes and rotate with the gripper. Agent jogs are translation-only. Deterministic policy measures the proposed displacement
and height through the saved physical workspace calibration:

- above 100 mm physical height: maximum 50 mm physical displacement per command;
- at or below 100 mm: maximum 10 mm physical displacement per command;
- a target below the calibrated ground plane is rejected.

The SDK's normal joint, Cartesian path, following-error, effort, fault, calibration, and
completion guards remain authoritative.

Move to the built-in calibrated default Sleep posture:

```bash
soarm101 agent sleep
```

The default uses the smoother calibration-relative wrist position at 75% of the executable
wrist-flex range. Use the historical fully folded wrist-up override only when explicitly
needed:

```bash
soarm101 agent sleep-up
```

STOP/HOLD is always available:

```bash
soarm101 agent stop
```

Every successful agent motion ends holding its reached position. The agent does not relax
the arm. Torque release is a human-only action.

## Human direction words

Do not infer human directions from the raw model X/Y/Z signs or from saved poses.

Read `soarm101 agent capabilities` and use its `world_directions` block. The values come
from the already-measured paper/workspace calibration:

- physical +X is A -> B (right); physical -X is left;
- physical +Y is A -> D / B -> C (forward, toward the top/long edge of the calibrated paper);
- physical -Y is back;
- physical +Z is D -> UP (up); physical -Z is down.

`model_delta_mm_per_physical_mm` gives the model/world XYZ delta corresponding to one
millimeter of each physical human direction. To move a requested physical distance, multiply
that direction vector by the requested distance in millimeters and pass the resulting XYZ
values to the existing command:

```bash
soarm101 agent jog --x-mm DX --y-mm DY --z-mm DZ
```

Example: if `world_directions.model_delta_mm_per_physical_mm.left` is
`[0.10, 0.98, -0.02]`, moving 10 mm left means jogging
`[1.0, 9.8, -0.2]` mm in model/world XYZ.

If `world_directions.available` is false, do not guess the mapping.

## Object-to-container task strategy

Use the overhead camera as the authoritative wide view for understanding task state and
for deciding whether the task is complete. Use the wrist camera when a closer view would
help with approach, grasp, container-edge clearance, release, or other fine motion.

The named observation poses are affordances, not a required sequence. Choose viewpoints
that provide useful information. Prefer saved poses for large repositioning and bounded
jogs for small corrections.

Re-observe after consequential manipulation when the result is uncertain. Do not claim
success from the intended motion alone.

## Completion contract

The task is **finished only when a fresh overhead-camera image clearly shows the target
object inside the target container**.

For this task, "inside" means the visible object is contained within the container's
interior footprint/opening rather than merely touching the rim, overlapping the outside
edge, sitting beside the container, or being too occluded to judge. If the fresh overhead
view is ambiguous, the task is not yet finished; obtain another view or continue the task.

After confirming completion, the final response must be exactly one JSON object:

```json
{
  "task_status": "finished",
  "evidence_camera": "overhead",
  "evidence_path": "/path/from/the/final/fresh/capture.jpg"
}
```

If the task cannot be completed, return:

```json
{
  "task_status": "not_finished",
  "reason": "brief concrete reason",
  "last_evidence_path": "/path/to/the/most/relevant/recent/image.jpg"
}
```

Never emit `"task_status": "finished"` without a fresh overhead image that supports it.
