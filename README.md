# SO-ARM101 Motion SDK

A high-level Python motion-control SDK for the SO-ARM101 robot arm.

`soarm101-motion-sdk` builds on LeRobot's SO-ARM101 hardware support and exposes a stable, application-friendly interface for joint motion, trajectories, kinematics, Cartesian positioning, safety, and diagnostics.

> **Project status:** Early development. Motion commands must be tested at low speed with a clear workspace and an accessible power disconnect.

## Scope

This repository contains reusable robot-arm control only. Camera operation, subject tracking, cinematic shot planning, video recording, OBS integration, and show orchestration belong in applications such as Robo Cam.

## Architecture

```text
Applications / Robo Cam
        |
        v
SOARM101 application API
        |
        v
RobotBackend protocol
   |              |
   v              v
Mock backend   LeRobot backend
                    |
                    v
             Feetech motor bus
```

The public API is inspired in part by the UFACTORY xArm Python SDK. This project is not affiliated with or endorsed by UFACTORY. It copies the developer experience where useful, not xArm's controller-specific implementation.

## Installation

```bash
uv sync --extra dev
```

LeRobot hardware support is optional:

```bash
uv sync --extra dev --extra lerobot
```

## Mock quick start

```python
from soarm101_motion import SOARM101
from soarm101_motion.backends import MockSOARM101Backend

arm = SOARM101(backend=MockSOARM101Backend())
arm.connect()
print(arm.get_joint_positions())
arm.move_joints({"shoulder_pan": 0.25})
arm.relax()
arm.disconnect()
```

## Current API

- `connect()` and `disconnect()`
- `get_joint_positions()` and `get_hardware_state()`
- validated direct `move_joints()` commands
- `stop()`, `hold()`, and `relax()`
- mock backend for local development and CI
- isolated LeRobot backend boundary

See [`PLAN.md`](PLAN.md) for the implementation roadmap and [`docs/safety.md`](docs/safety.md) before operating hardware.
