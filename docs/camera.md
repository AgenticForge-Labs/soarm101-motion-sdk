# Camera

The SO-ARM101 Motion SDK includes a small local USB-camera layer for bench observation.
It deliberately stops at device ownership, capture settings, live frames, and still-image
persistence. Object detection, tracking, scene understanding, camera-to-robot calibration,
and task reasoning remain higher-level responsibilities.

## Shared settings

CLI and GUI read the same persisted settings from:

```text
~/.config/soarm101/camera.json
```

The settings are:

- device index or path, such as `/dev/video0`;
- width and height;
- requested frames per second;
- four-character capture codec, normally `MJPG` for USB cameras;
- optional horizontal mirroring;
- GUI auto-start preference; and
- snapshot output directory.

The GUI owns at most one live camera session. The Camera tab configures it and Teleoperation
renders the same frames. Do not open the same camera in another application or CLI command
while the GUI stream is active.

## CLI

Install camera support without the desktop GUI:

```bash
pip install -e ".[camera]"
```

Find likely devices:

```bash
soarm101 camera list
```

Save the shared configuration:

```bash
soarm101 camera configure \
  --device /dev/video0 \
  --width 1280 \
  --height 720 \
  --fps 30 \
  --fourcc MJPG
```

Inspect it:

```bash
soarm101 camera show --json
```

Acquire one fresh observation:

```bash
soarm101 camera capture --json
```

The JSON result includes the saved path, timestamp, dimensions, and device. This command is
the intended primitive for slow agentic observe → reason → move → observe loops. The agent
does not need to discover OpenCV, V4L2, or camera-driver details itself.

## GUI

The **Camera** tab is the single settings surface. It provides device selection/discovery,
resolution, FPS, FourCC, mirroring, snapshot folder, start/stop, and picture capture.

The **Teleoperation** tab shows the same live stream and exposes only session-level start/stop
and picture capture. Camera configuration stays in Camera so changes propagate consistently
instead of creating per-tab copies.

Capture runs on a background thread. A delayed USB frame should not block robot controls or
the Qt event loop.

## Agent boundary

A constrained agent may be given both camera and motion CLI commands:

```text
soarm101 camera capture --json
        ↓
multimodal reasoning
        ↓
soarm101 jog / move-joints / gripper
        ↓
soarm101 camera capture --json
```

Motion commands continue through the existing calibration, workspace, rate, acceleration,
following-error, fault, effort, and provenance guards. Camera frames do not authorize motion
and camera code never writes motor registers.

## Device notes

On Linux, prefer a stable `/dev/v4l/by-id/...` path when one is available; `/dev/videoN`
numbers can change after replugging devices. The current discovery command lists local
`/dev/video*` nodes, so a stable by-id path can be entered manually in Camera or passed to
`camera configure`.

Requested width, height, FPS, and FourCC are negotiated with the camera driver. The GUI
reports the format OpenCV says it actually opened. Validate the exact camera and USB port on
the target workstation before relying on timing or image geometry.
