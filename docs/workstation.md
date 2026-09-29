# Workstation profile

The SO-ARM101 Motion SDK keeps machine-local hardware addressing in one shared profile:

```text
~/.config/soarm101/workstation.json
```

The GUI, CLI, and external agents consume the same profile. It contains local references,
not authoritative calibration contents.

## Stored fields

```json
{
  "schema_version": 1,
  "follower": {
    "port": "/dev/ttyACM0",
    "robot_id": "so101",
    "calibration": "~/.config/soarm101/calibration/so101.json"
  },
  "leader": {
    "port": "/dev/ttyACM1",
    "robot_id": "so101-leader",
    "calibration": "~/.config/soarm101/calibration/so101-leader.json"
  },
  "selected_camera": "overhead",
  "cameras": {
    "overhead": {
      "device": "/dev/v4l/by-id/...",
      "width": 1280,
      "height": 720,
      "fps": 30.0,
      "fourcc": "MJPG",
      "mirror": false,
      "auto_start": true,
      "snapshot_dir": "~/.local/share/soarm101/captures"
    }
  }
}
```

Calibration data remain authoritative in the files under
`~/.config/soarm101/calibration/`. The workstation profile only remembers which
robot/calibration identity is associated with each local arm role.

## GUI behavior

A successful physical follower or leader connection saves its selected port, robot ID, and
calibration-file reference.

The Camera tab edits a registry of uniquely named camera profiles. Names should describe
their physical role, for example:

- `overhead`
- `wrist`
- `side`

More than one named camera can stream concurrently. Each physical camera device may belong to
only one saved profile. Camera and Teleoperation views reuse those named sessions instead of
opening duplicate handles.

## CLI

Inspect the complete profile:

```bash
soarm101 workstation show --json
```

Configure arm addressing without opening hardware:

```bash
soarm101 workstation arm follower \
  --port /dev/ttyACM0 \
  --robot-id so101 \
  --json
```

Camera commands use the same profile:

```bash
soarm101 camera list --json
soarm101 camera configure --name overhead --device /dev/v4l/by-id/... --json
soarm101 camera configure --name wrist --device /dev/v4l/by-id/... --json
soarm101 camera capture --name overhead --json
soarm101 camera capture --all --json
```

On Linux, camera discovery prefers stable `/dev/v4l/by-id/*-video-index0` paths when
available. This is preferred over `/dev/videoN`, whose numbering may change after replugging
or rebooting.

## Migration

If `workstation.json` does not yet exist but the legacy
`~/.config/soarm101/camera.json` file does, its single camera settings are loaded as a camera
profile named `camera`. Saving from the new GUI/CLI writes the workstation profile.

## Environment override

For tests or isolated automation, set:

```bash
export SOARM101_WORKSTATION_CONFIG=/path/to/workstation.json
```

This changes only the workstation profile path. It does not relocate calibration files unless
the profile explicitly references alternate calibration paths.
