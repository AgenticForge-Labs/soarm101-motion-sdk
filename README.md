# SO-ARM101 Motion SDK

A self-contained, high-level Python motion-control SDK for the SO-ARM101.

It uses the official Feetech Python SDK directly for the six STS3215 motors and provides smooth joint motion, native forward and inverse kinematics, Cartesian linear movement, calibration compatibility, tool/TCP handling, diagnostics, and simulation. **ROS and LeRobot are not runtime dependencies.**

> **Status:** Alpha hardware software. The protocol, motor IDs, calibration behavior, joint geometry, and limits are based on the official SO-ARM101, Feetech, and LeRobot implementations. Automated tests cover simulation and the hardware adapter with a fake Feetech transport. A physical-arm smoke test is still required before claiming a release is hardware-validated. Start with no payload, low speed, a clear workspace, and accessible power.

## Architecture

```text
Robo Cam / applications
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

First find the serial port:

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

The GUI command requires the `simulation` extra.

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

`set_position` follows xArm units: millimeters for position/speed/acceleration; `is_radian` controls orientation angles only. These are convenience aliases, not UFACTORY controller return-code compatibility.

## Safety model

- No movement occurs during construction or connection.
- Torque is enabled only through `enable()` or `auto_enable_torque=True`.
- Every joint target is checked against official URDF limits.
- Point-to-point moves use a host-side minimum-jerk profile.
- Linear moves are fully waypointed, solved, and validated before the first motor command.
- Every low-level command is step-limited.
- Software stop is not a certified emergency stop; keep physical power accessible.

See [docs/hardware.md](docs/hardware.md), [docs/kinematics.md](docs/kinematics.md), and [docs/simulation.md](docs/simulation.md).
