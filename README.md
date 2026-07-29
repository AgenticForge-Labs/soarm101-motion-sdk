# SO-ARM101 Motion SDK

A self-contained, high-level Python motion-control SDK for the SO-ARM101.

It uses the Feetech Python SDK directly for the six STS3215 motors and provides smooth joint motion, native forward and inverse kinematics, Cartesian linear movement, calibration compatibility, tool/TCP handling, diagnostics, conservative workspace checks, guided hardware validation, and simulation. **ROS and LeRobot are not runtime dependencies.**

> **Status:** Alpha hobby-arm software. Automated tests cover simulation and a fake Feetech transport. Physical-arm validation is still required on each printed assembly. Start with no payload, low speed, a clear workspace, and accessible physical power.

## Architecture

```text
Robo Studio / Robo Puppeteer / applications
        |
        v
SOARM101 public API
  joint + linear moves, FK/IK, tools, safety envelope
        |
        v
SO101HardwareBackend
   |                    |
   v                    v
FeetechBackend       SimulationBackend
   |                    |-- deterministic in-memory
   v                    `-- optional PyBullet GUI
ftservo-python-sdk 2.0.0
```

LeRobot remains useful as a calibration-format and behavioral reference. Existing LeRobot SO-101 calibration JSON files are discovered and loaded automatically.

## Agentic Forge integration

Robo Director is the primary source of cross-repository integration requirements. This SDK is a library, not a Director service. Robo Studio and Robo Puppeteer wrap it and expose Director actions, health, terminal events, and resource claims.

The SDK publishes stable integration metadata for its units, default base frame, nonblocking motion handles, physical resource identity, software-stop classification, and TCP ownership. See [INTEGRATION.md](INTEGRATION.md).

Both consumer adapters must use the same resource identifier, such as `motion-platform:soarm101`, so Director prevents a camera move and a character gesture from commanding one arm concurrently.

## Install

```bash
pip install soarm101-motion-sdk
```

For visual simulation:

```bash
pip install "soarm101-motion-sdk[simulation]"
```

For development:

```bash
uv sync --extra dev --extra simulation
```

## Simulation first

```bash
soarm101 sim-demo
soarm101 sim-demo --gui --realtime
```

The simulator runs the same planner, IK, tool, cancellation, and workspace-envelope code used by hardware.

## One-time hardware setup

New loose motors must be configured one at a time. Only one servo may be connected during each step:

```bash
soarm101 setup-motors --port /dev/ttyACM0
```

Then reconnect the complete six-motor daisy chain and run a truly read-only diagnostic:

```bash
soarm101 diagnose --port /dev/ttyACM0 --robot-id forge-arm --allow-uncalibrated
```

Normal connections do **not** rewrite EEPROM or PID settings. Apply the recommended settings explicitly once:

```bash
soarm101 configure --port /dev/ttyACM0 --robot-id forge-arm
```

Calibrate after setup/configuration:

```bash
soarm101 calibrate \
  --port /dev/ttyACM0 \
  --robot-id forge-arm \
  --seconds 30 \
  --export-lerobot
```

## Supervised first powered test

Run one joint at a time with no payload and physical power accessible:

```bash
soarm101 smoke-test \
  --port /dev/ttyACM0 \
  --robot-id forge-arm \
  --joint shoulder_pan
```

Repeat for `shoulder_lift`, `elbow_flex`, `wrist_flex`, and `wrist_roll` only after confirming each direction.

## Python control

```python
from soarm101_motion import Pose, SOARM101

with SOARM101(port="/dev/ttyACM0", robot_id="forge-arm") as arm:
    arm.enable()
    arm.move_joints(
        [0.0, -0.4, 0.7, 0.0, 0.2],
        speed=0.35,
        acceleration=0.9,
    )
    arm.tool.open()
    arm.tool.close()

    pose = arm.get_position()
    target = Pose(pose.position + [0.02, 0.0, 0.0], pose.rotation)
    arm.move_linear(target, orientation_mode="position_only", speed=0.02)
    arm.relax()
```

The stock gripper is an `SO101Gripper` tool, not a sixth pose joint. A camera or later parallel gripper can define another TCP without changing the five-joint arm model.

## Kinematics validation

Position the unpowered arm at a measured checkpoint, measure the TCP in millimeters, and record the comparison:

```bash
soarm101 kinematics-check \
  --port /dev/ttyACM0 \
  --robot-id forge-arm \
  --sample home \
  --x-mm 391.4 --y-mm 0 --z-mm 226.5 \
  --output kinematics-validation.jsonl
```

Use several diverse poses before trusting larger Cartesian moves. See [docs/validation.md](docs/validation.md).

## xArm-inspired convenience API

```python
arm.set_servo_angle(
    angle=[0, -25, 40, 0, 15],
    is_radian=False,
    speed=25,
    mvacc=50,
    wait=True,
)

arm.set_position(
    x=200,
    y=0,
    z=150,
    pitch=40,
    speed=30,
    mvacc=100,
    wait=True,
)
```

`set_position` follows xArm units: millimeters for position/speed/acceleration; `is_radian` controls orientation angles only. These are convenience aliases, not UFACTORY controller or return-code compatibility.

## Safety model

- Construction and ordinary connection never enable torque.
- Ordinary connection and diagnostics do not rewrite motor configuration.
- Direct hardware motion writes are rejected while torque is disabled.
- Torque enable first latches every servo's measured position as its goal.
- Joint targets are checked against model and calibrated limits.
- Joint and Cartesian paths are preplanned and checked for speed, acceleration, step size, gross floor/base collisions, reach, and coarse self-clearance.
- Active motion monitors following error, unexpected direction, motor status, and command timing.
- Blocking and nonblocking moves share cancellation; `stop()`/`software_stop()` hold the arm and tool.
- `wait=True` verifies measured completion.
- Calibration restores EEPROM after failure when possible.
- There is deliberately no `emergency_stop()` method: software stop is not a physical E-stop.

See [INTEGRATION.md](INTEGRATION.md), [hardware setup](docs/hardware.md), [kinematics](docs/kinematics.md), [validation](docs/validation.md), [simulation](docs/simulation.md), and [safety](docs/safety.md).
