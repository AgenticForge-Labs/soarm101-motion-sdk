# Agent-facing SO-ARM101 CLI

This is the small CLI surface intended for an observe-reason-act experiment. It is not a
replacement for `soarm101 --help`; current CLI help and SDK source are authoritative if
this document ever disagrees with them.

Values such as `ROBOT_PORT` and `ROBOT_ID` come from `setup.local.json`.

## Inspect before motion

List likely camera devices:

```bash
soarm101 camera list --json
```

Read the calibrated robot state without configuration writes:

```bash
soarm101 read --port ROBOT_PORT --robot-id ROBOT_ID
```

Read motor diagnostics without configuration writes:

```bash
soarm101 diagnose --port ROBOT_PORT --robot-id ROBOT_ID --json
```

## Capture an image

Capture from a specific USB camera without changing persisted GUI camera settings:

```bash
soarm101 camera capture \
  --device /dev/v4l/by-id/CAMERA \
  --width 1280 \
  --height 720 \
  --fps 30 \
  --fourcc MJPG \
  --output observation.jpg \
  --json
```

Add `--mirror` only when that camera is configured as mirrored. Use
`capture_observation.py` when a fresh frame from every configured camera is wanted.

## Relative Cartesian motion

`jog` performs one guarded Cartesian linear jog. Translation arguments are millimeters.
The default frame is `world`; `--frame tool` requests a TCP-relative jog.

```bash
soarm101 jog \
  --port ROBOT_PORT \
  --robot-id ROBOT_ID \
  --frame world \
  --x-mm 2 \
  --y-mm 0 \
  --z-mm 0 \
  --speed-mm-s 10 \
  --acceleration-mm-s2 40 \
  --yes
```

The SDK validates the complete planned Cartesian path before motor commands. Prefer small
increments during agentic manipulation.

## Absolute Cartesian motion

`move-linear` targets an absolute world TCP position in millimeters:

```bash
soarm101 move-linear \
  --port ROBOT_PORT \
  --robot-id ROBOT_ID \
  --x-mm 250 \
  --y-mm 50 \
  --z-mm 180 \
  --orientation-mode position_only \
  --speed-mm-s 10 \
  --acceleration-mm-s2 40 \
  --yes
```

The five-axis arm cannot realize every full orientation. The available orientation modes
are `compatible`, `position_only`, and `exact`. For visual pick-and-place experiments,
do not invent orientation constraints that the task does not require.

## Gripper

Open or close the stock gripper:

```bash
soarm101 gripper --port ROBOT_PORT --robot-id ROBOT_ID open --yes
soarm101 gripper --port ROBOT_PORT --robot-id ROBOT_ID close --yes
```

A normalized target from `0` through `1` may be used instead of `open` or `close`:

```bash
soarm101 gripper --port ROBOT_PORT --robot-id ROBOT_ID 0.5 --yes
```

## Absolute joint motion

Use only when a joint-space move is actually appropriate:

```bash
soarm101 move-joints \
  --port ROBOT_PORT \
  --robot-id ROBOT_ID \
  J1 J2 J3 J4 J5 \
  --degrees \
  --yes
```

There are five pose joints; the gripper is controlled separately.

## Existing named poses

List and replay poses created through the shared SDK/GUI library:

```bash
soarm101 pose list --robot-id ROBOT_ID
soarm101 pose go --port ROBOT_PORT --robot-id ROBOT_ID home --yes
```

Do not assume a named pose exists until it has been listed.

## Operating rule

The normal loop is:

```text
capture fresh image(s)
        ↓
read state when useful
        ↓
reason
        ↓
issue one guarded motion/gripper action
        ↓
capture fresh image(s)
        ↓
repeat
```

Never infer physical success only from a command returning successfully. Command success
means the SDK accepted/executed the guarded operation; the camera observation determines
whether the intended physical effect occurred.
