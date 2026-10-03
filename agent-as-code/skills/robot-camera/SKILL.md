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

Open or close the gripper:

```bash
soarm101 agent gripper open
soarm101 agent gripper close
```

Make a bounded model/base-frame translational adjustment:

```bash
soarm101 agent jog --x-mm 5 --y-mm 0 --z-mm 0
```

Agent jogs are translation-only. Deterministic policy measures the proposed displacement
and height through the saved physical workspace calibration:

- above 100 mm physical height: maximum 50 mm physical displacement per command;
- at or below 100 mm: maximum 10 mm physical displacement per command;
- a target below the calibrated ground plane is rejected.

The SDK's normal joint, Cartesian path, following-error, effort, fault, calibration, and
completion guards remain authoritative.

Move to the built-in calibrated Sleep posture:

```bash
soarm101 agent sleep
```

STOP/HOLD is always available:

```bash
soarm101 agent stop
```

Every successful agent motion ends holding its reached position. The agent does not relax
the arm. Torque release is a human-only action.

## Human direction words for this workstation

For the current physical workstation orientation, use these verified human-direction
translations with the existing world-frame `agent jog` command:

- `left` = positive world/model Y
- `right` = negative world/model Y

Examples:

```bash
# move 10 mm left
soarm101 agent jog --x-mm 0 --y-mm 10 --z-mm 0

# move 10 mm right
soarm101 agent jog --x-mm 0 --y-mm -10 --z-mm 0
```

Do not infer `forward`, `back`, `up`, or `down` from model X/Y/Z yet. Those
human directions have not been physically verified for this workstation, and model axes
are not guaranteed to match physical table/up directions. If a task uses one of those
unverified words, inspect available known poses/cameras or report that the direction mapping
needs human confirmation rather than guessing.

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
