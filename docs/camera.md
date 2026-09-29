# Camera

The SO-ARM101 Motion SDK includes a local USB/UVC camera layer for bench observation. It
owns device discovery, named capture profiles, live frame sessions, and still-image
persistence. Object detection, tracking, scene understanding, camera-to-robot calibration,
and task reasoning remain higher-level responsibilities.

## Named cameras and shared workstation state

Camera profiles are stored with the arm connections in:

```text
~/.config/soarm101/workstation.json
```

Each camera has a stable logical name such as `overhead`, `wrist`, or `side`, plus:

- device index or path;
- width and height;
- requested frames per second;
- four-character capture codec, normally `MJPG`;
- optional horizontal mirroring;
- GUI auto-start preference; and
- snapshot output directory.

A physical device may belong to only one named profile.

The previous `~/.config/soarm101/camera.json` single-camera configuration is migrated as a
profile named `camera` when no workstation profile exists yet.

## Linux device discovery

`soarm101 camera list` prefers stable paths under:

```text
/dev/v4l/by-id/*-video-index0
```

when available. These names are preferable to `/dev/videoN`, which may change after reboot
or replugging. If stable by-id entries are unavailable, discovery falls back to local
`/dev/video*` nodes.

## CLI

Inspect discovered devices and configured names:

```bash
soarm101 camera list --json
soarm101 camera show --json
soarm101 camera show --name overhead --json
```

Create/update camera profiles:

```bash
soarm101 camera configure \
  --name overhead \
  --device /dev/v4l/by-id/CAMERA-video-index0 \
  --width 1280 \
  --height 720 \
  --fps 30 \
  --fourcc MJPG \
  --json
```

Select the default camera:

```bash
soarm101 camera select overhead
```

Capture one named camera:

```bash
soarm101 camera capture --name overhead --json
```

Capture every configured camera sequentially:

```bash
soarm101 camera capture --all --json
```

CLI capture opens camera devices only for the duration of the capture. Do not run a CLI
capture against a device currently owned by the GUI.

## GUI

The **Camera** tab is the settings surface for named cameras. Its compact setup panel supports:

- selecting/renaming a saved camera profile;
- creating and deleting profiles;
- an explicit USB-device dropdown populated from local discovery;
- per-camera resolution/FPS/FourCC/mirroring/snapshot settings;
- per-camera auto-start;
- starting/stopping the selected camera;
- starting/stopping all configured cameras; and
- capturing a still from the selected camera.

Camera discovery runs when the tab is built and can also be refreshed manually. The device
selector is deliberately non-editable in the GUI so a discovered camera looks like a real
choice rather than an ambiguous text field; custom device paths remain configurable through
the CLI/workstation profile.

The lower Camera-tab area shows every configured camera simultaneously: one camera uses the
full preview area, two cameras split side-by-side, and three or more use a two-column grid.
Each card is labeled with its logical camera name and live/stopped state.

The GUI may own multiple live camera sessions concurrently, one worker per named physical
device. Camera and Teleoperation views consume those shared named sessions rather than opening
their own duplicate device handles.

The **Teleoperation** tab has a camera-name selector. Changing the selector changes only which
already-named stream is displayed; it does not redefine camera settings.

## Agent boundary

An external agent can discover the exact machine-local names through the same deterministic
CLI surface:

```bash
soarm101 workstation show --json
soarm101 camera list --json
soarm101 camera capture --name overhead --json
soarm101 camera capture --name wrist --json
```

The camera layer returns images and metadata only. It does not identify objects, infer task
state, authorize motion, or bypass the motion SDK.

Motion continues through calibration, workspace, rate, acceleration, following-error,
fault, effort, and provenance guards.

## Device notes

Requested width, height, FPS, and FourCC are negotiated with the camera driver. Each GUI
worker reports the format OpenCV says it actually opened. USB bandwidth and camera drivers
can limit concurrent streams even when individual cameras work separately, so validate the
exact camera set on the target workstation.
