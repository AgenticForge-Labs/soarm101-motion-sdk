# SO-ARM101 Motion SDK

A self-contained, high-level Python motion-control SDK for the SO-ARM101.

It uses the official Feetech Python SDK directly for the six STS3215 motors and provides smooth joint motion, native forward and inverse kinematics, Cartesian linear movement, calibration compatibility, tool/TCP handling, diagnostics, and simulation. **ROS and LeRobot are not runtime dependencies.**

> **Status:** Alpha hardware software. Automated tests cover simulation and the hardware adapter with a fake Feetech transport. A physical-arm smoke test is still required before claiming a hardware-validated release. Start with no payload, low speed, a clear workspace, and accessible power.

## Architecture

```text
Robo Studio / Robo Puppeteer / applications
        |
        v
SOARM101 public API
  joint + linear moves, FK/IK, trajectories, tools
        |
        v
SO101HardwareBackend
   |                    |
   v                    v
FeetechBackend       SimulationBackend
   |                    |-- deterministic in-memory
   v                    `-- optional PyBullet GUI
ftservo-python-sdk
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

## Hardware quick start

First find the serial port and diagnose the arm:

```bash
soarm101 ports
soarm101 diagnose --port /dev/ttyACM0 --allow-uncalibrated
```

Use an existing LeRobot calibration or calibrate:

```bash
soarm101 calibrate --port /dev/ttyACM0 --robot-id forge-arm --export-lerobot
```

Then control the arm:

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

The stock gripper is an `SO101Gripper` tool, not a sixth arm pose joint. A camera or later parallel gripper can define another TCP without changing the five-joint arm model.

## Simulation

The deterministic simulator runs the same planner and IK code as hardware:

```python
from soarm101_motion import SOARM101

with SOARM101.simulated() as arm:
    arm.enable()
    arm.move_joints([0.2, -0.4, 0.6, 0.1, 0.0])
    print(arm.get_position())
```

Run the complete demo:

```bash
soarm101 sim-demo
soarm101 sim-demo --gui --realtime
```

## Safety model

- No movement occurs during construction or connection.
- Direct hardware motion writes are rejected while torque is disabled.
- Torque enable latches each measured position as its goal to prevent stale-goal jumps.
- Every target is checked against model and calibrated limits.
- Joint and linear moves are fully planned and validated before hardware commands begin.
- Blocking and nonblocking moves share cancellation behavior.
- Completion uses measured feedback rather than transmission timing alone.
- Software stop is not a certified emergency stop; keep physical power accessible.

See [INTEGRATION.md](INTEGRATION.md), [docs/hardware.md](docs/hardware.md), [docs/kinematics.md](docs/kinematics.md), [docs/simulation.md](docs/simulation.md), and [docs/safety.md](docs/safety.md).
