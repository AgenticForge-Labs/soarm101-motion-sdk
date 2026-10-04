---
name: soarm101-openshell-hermes-robot-camera
description: Operate the bounded SO-ARM101 robot and named cameras from Hermes inside the canonical OpenShell sandbox.
---

# SO-ARM101 OpenShell robot + camera skill

You are running inside an isolated OpenShell sandbox. The physical robot and cameras remain
on the trusted host. Use only the local `robotctl.py` client for robot and camera actions.

Do not search for, import, or invoke an unrestricted robot SDK. Do not attempt to arm,
disarm, relax, calibrate, configure, write servo registers, mount host devices, or bypass
the broker. Human authorization for motion is established outside the sandbox. Read-only
capabilities, state, and named-camera capture remain available without motion authority. If
authority is absent or expired, do not request motion; continue observation-only work that
the task permits. If the task requires motion, report that motion cannot continue.

## Discover first

Start with:

```bash
python3 robotctl.py capabilities
python3 robotctl.py state
```

The capability response is authoritative for available poses, configured cameras, current
authority, and measured human-direction guidance.

## Cameras

Capture only configured named cameras:

```bash
python3 robotctl.py capture overhead
python3 robotctl.py capture wrist
```

A capture is written under the sandbox observation directory and the returned bytes are
verified against the trusted broker SHA-256 before the file is saved.

Use Hermes `vision_analyze` on the fresh local image path before making visual claims.
Do not infer scene state from filenames, requested motion, or prior images.

Use overhead for broad workspace/status/completion views when configured. Use wrist when a
close view improves approach, grasp, edge clearance, placement, or inspection.

## Saved poses

Only broker-exposed `agent_*` poses may be used:

```bash
python3 robotctl.py go-pose agent_start_overhead
```

Treat saved poses as coarse affordances, not as a mandatory sequence.

## Relative joint motion

Adjust one named arm joint by a bounded relative angle:

```bash
python3 robotctl.py joint shoulder_pan --delta-deg 5
python3 robotctl.py joint shoulder_lift --delta-deg -5
python3 robotctl.py joint elbow_flex --delta-deg 5
python3 robotctl.py joint wrist_flex --delta-deg -5
python3 robotctl.py joint wrist_roll --delta-deg 10
```

Prefer small increments and re-observe after consequential changes. The broker/SDK enforce
the allowed joint, per-command angle limit, calibrated limits, workspace/path checks, and
hold behavior.

## World-frame motion

For table/workspace directions, read `capabilities.world_directions`. Do not guess raw
model XYZ signs from saved poses or intuition.

The paper/workspace calibration defines physical axes and reports
`model_delta_mm_per_physical_mm` for right/left, forward/back, and up/down. Multiply the
reported direction vector by the desired physical distance in millimeters, then pass that
model/world XYZ delta to:

```bash
python3 robotctl.py jog --frame world --x-mm DX --y-mm DY --z-mm DZ
```

If `world_directions.available` is false, do not invent a physical direction mapping.

## Tool-frame motion

For a correction expressed from the gripper's own current perspective, use:

```bash
python3 robotctl.py jog --frame tool --x-mm DX --y-mm DY --z-mm DZ
```

Tool X/Y/Z are the current TCP axes and rotate with the gripper. Use small tool-frame moves
for approach/retract/lateral corrections.

Agent Cartesian jogs are translation-only. The broker applies measured physical-height and
displacement policy in addition to the Motion SDK's ordinary motion guards.

## Gripper

```bash
python3 robotctl.py gripper open
python3 robotctl.py gripper close
```

The physical targets remain inset from calibrated mechanical endpoints.

## Hold / stop / sleep

```bash
python3 robotctl.py stop
python3 robotctl.py sleep
```

Successful motions remain torque-held. `stop` means STOP/HOLD, not torque release. Relax is
not available to the sandbox.

## Operating pattern

Use an observe -> bounded action -> re-observe loop when task state depends on the physical
scene. Prefer the smallest useful action. A rejected action is evidence that the requested
motion is not currently authorized or safe; do not route around it.

Do not claim task completion from intended motion alone. Use fresh camera evidence when the
task has a visible completion condition.

If TASK.md specifies an output file or exact completion JSON, follow that contract exactly.
Otherwise report the outcome in your final response with the last relevant observation path.
