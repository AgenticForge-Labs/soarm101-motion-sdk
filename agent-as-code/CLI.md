# Agent-facing robot interface

Inside the OpenShell sandbox, physical interaction is exposed only through `robotctl`.
The host-side executor translates these requests into the public `soarm101` CLI and the
Motion SDK remains authoritative for motion and camera behavior.

Use `robotctl --help` or a subcommand's `--help` for syntax.

## Check capabilities

```bash
robotctl health
```

This reports the robot ID, primary camera name, auxiliary camera names, available actions,
and the host executor's maximum per-request jog limits. It does not move the robot or
open a camera.

## Observe

```bash
robotctl observe --label initial
```

This captures one fresh frame from every enabled host camera and writes the transferred
files under:

```text
/workspace/runs/<observation-id>/
```

The command prints JSON containing the manifest and image paths. The primary camera is
listed first.

Inspect those actual image files with your available vision capability before making a
visually grounded decision.

## Read robot state

```bash
robotctl state
```

The host maps this to the SDK's read-only calibrated state command.

Motor diagnostics:

```bash
robotctl diagnose
```

## Relative Cartesian motion

`jog` requests one guarded relative Cartesian move:

```bash
robotctl jog --frame world --x-mm 2
robotctl jog --frame world --y-mm -2
robotctl jog --frame world --z-mm 2
```

Tool-frame motion is available when it is useful:

```bash
robotctl jog --frame tool --z-mm -2
```

Rotation can be included:

```bash
robotctl jog --yaw-deg 2
```

Optional controls:

```bash
robotctl jog \
  --x-mm 2 \
  --orientation-mode compatible \
  --speed-mm-s 5 \
  --acceleration-mm-s2 20
```

The executor rejects a request that exceeds its configured vector translation, vector
rotation, speed, or acceleration ceiling before the Motion SDK sees it. The SDK then
validates the complete requested path and its normal hardware safety conditions.

Large physical moves should therefore emerge from repeated observe/reason/small-action
cycles rather than one unrestricted command.

## Gripper

```bash
robotctl gripper open
robotctl gripper close
robotctl gripper 0.5
```

Normalized positions must be between 0 and 1.

## Deliberately unavailable

The sandbox interface does not currently expose:

- arbitrary shell passthrough to the host;
- direct serial, USB, OpenCV, or servo access;
- absolute joint commands;
- arbitrary absolute Cartesian targets;
- calibration/configuration writes;
- trajectory/program authoring;
- raw motor registers.

Those may exist in the host SDK/CLI, but they are outside this first benchmark capability
contract.

## Normal control loop

```text
robotctl observe
        ↓
inspect image(s)
        ↓
reason
        ↓
robotctl jog / gripper
        ↓
robotctl observe
        ↓
repeat
```

A successful command means the host executor and SDK accepted/executed the request. It
does **not** prove that the requested physical task effect happened. Confirm task effects
from fresh observation.
