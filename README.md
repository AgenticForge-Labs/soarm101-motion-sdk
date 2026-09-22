# SO-ARM101 Motion SDK

A focused Python motion SDK for the five-axis SO-ARM101 follower arm and its stock gripper.

It talks directly to the six Feetech STS3215 servos and provides joint motion, FK/IK, Cartesian linear movement, tools/TCPs, diagnostics, calibration compatibility, conservative workspace checks, simulation, and an optional PySide6 controller. **ROS and LeRobot are not runtime dependencies.**

> **Status:** Beta hobby-arm software. Simulation and fake-transport tests are automated; each printed arm still needs a supervised no-payload smoke test with physical power accessible.

## Install from this repository

The package is not yet published as a release on PyPI. Install the checked-out repository:

```bash
git clone https://github.com/AgenticForge-Labs/soarm101-motion-sdk.git
cd soarm101-motion-sdk

python3 -m venv .venv
source .venv/bin/activate       # Windows: .venv\Scripts\activate
python -m pip install --upgrade pip
pip install -e .
```

Optional interfaces:

```bash
pip install -e ".[simulation]"   # PyBullet visual simulation
pip install -e ".[gui]"          # PySide6 desktop controller
```

## Try simulation

```bash
soarm101 sim-demo
soarm101 sim-demo --gui --realtime
```

The simulator uses the same planner, IK, tool, cancellation, and workspace-envelope code as hardware.

## Set up an assembled arm

For an arm whose six motor IDs are already assigned, one guided command performs read-only diagnostics, applies the recommended motor configuration, keeps a usable EEPROM calibration or guides a new calibration, and saves the calibration file:

```bash
soarm101-setup --robot-id forge-arm
```

The serial port is selected automatically when exactly one adapter is connected. Pass `--port /dev/ttyACM0` or `--port COM7` when more than one is present.

The setup command never enables torque. It ends by printing the exact first smoke-test command.

### New loose motors

Fresh motors normally share a factory ID and must be assigned one at a time before the arm is fully daisy-chained:

```bash
soarm101 setup-motors --port /dev/ttyACM0
soarm101-setup --port /dev/ttyACM0 --robot-id forge-arm
```

Connect exactly one motor whenever `setup-motors` asks. Common factory settings are detected quickly; unusual settings can be supplied with `--initial-id` and `--initial-baudrate`.

## First powered test

Remove payloads, clear the workspace, and keep the physical power switch or plug within reach:

```bash
soarm101 smoke-test \
  --port /dev/ttyACM0 \
  --robot-id forge-arm \
  --joint shoulder_pan
```

Repeat for `shoulder_lift`, `elbow_flex`, `wrist_flex`, and `wrist_roll` only after confirming each previous joint moves in the expected direction and returns cleanly.

## PySide6 controller

Install the optional GUI and launch it:

```bash
pip install -e ".[gui]"
soarm101-gui --robot-id forge-arm
```

Or test the complete interface without hardware:

```bash
soarm101-gui --simulation
```

The GUI provides:

- Setup, Control, Teach, Trajectories, and Run workspaces;
- live mechanical-stop midpoint calibration using the same backend as the CLI;
- persistent user-recorded Home and Rest poses;
- a second read-only leader/controller-arm session and selectable teaching source;
- named taught points replayable as joint or Cartesian linear moves;
- 50 Hz exact trajectory recording with gripper and optional effort/current diagnostics;
- non-destructive raw/edited trajectory storage and a lightweight timeline editor;
- validated replay with crop selection, speed scaling, and safe movement to the clip start;
- advanced non-destructive trajectory editing: smoothing, delete/splice, holds, keyframes,
  markers, and repeated clips;
- semantic motion primitives that reference saved trajectories for higher-level consumers;
- a persistent sequence editor/runner combining points, Home/Rest, gripper actions, waits,
  trajectories, and motion primitives, with step execution, repeats, speed scaling,
  step-boundary pause/resume, and STOP/HOLD;
- guarded leader-to-follower joint teleoperation with relative/clutch-safe or
  absolute calibrated mapping, optional gripper mirroring, and a conservative 10 Hz
  default stream rate (5/10/20/50 Hz selectable; higher rates require hardware validation);
- calibrated slider limits after calibration;
- measured and target values for all five joints, with degree sliders;
- guarded absolute joint moves;
- measured world/base XYZ, roll, pitch, and yaw;
- absolute world-pose linear moves;
- direct ±XYZ and ±roll/pitch/yaw jog buttons;
- world-frame or current-tool-frame jog semantics;
- tool-space translations along the current gripper axes;
- compatible, position-only, or exact orientation modes;
- gripper open, close, and normalized positioning;
- connect, torque-enable, software stop/hold, relax, and live status;
- a persistent worker-thread session so the window remains responsive while SDK motion runs.

Every Cartesian GUI jog uses the normal `move_linear()` planner. The GUI does not bypass joint, calibration, workspace, following-error, timing, or fault checks. Live teleoperation likewise uses the guarded streaming API: every sample is checked for calibrated/model limits, command step, speed, acceleration, workspace path, following error, faults, and effort trips.

The streaming/teleoperation implementation is covered by simulation and automated tests but has **not yet been physically validated on this arm**. Follow `TESTING.md` before relying on it with hardware.

Matching CLI controls are available for scripting and troubleshooting:

```bash
# Absolute five-joint target in degrees
soarm101 move-joints --port /dev/ttyACM0 --robot-id forge-arm \
  --degrees 0 -20 35 0 10 --yes

# Move 5 mm along the current tool X axis with a linear Cartesian path
soarm101 jog --port /dev/ttyACM0 --robot-id forge-arm \
  --frame tool --x-mm 5 --yes

# Rotate 2 degrees around world yaw
soarm101 jog --port /dev/ttyACM0 --robot-id forge-arm \
  --frame world --yaw-deg 2 --yes

soarm101 gripper --port /dev/ttyACM0 --robot-id forge-arm open --yes
soarm101 gripper --port /dev/ttyACM0 --robot-id forge-arm close --yes
```

The same GUI can also be launched through `soarm101 gui`.

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
    target = Pose(pose.position + [0.01, 0.0, 0.0], pose.rotation)
    arm.move_linear(target, orientation_mode="position_only", speed=0.01)

    arm.relax()
```

The stock gripper is a tool actuator, not a sixth pose joint. Camera and future parallel-gripper tools can define their own TCPs without changing the five-joint arm model.

## Useful commands

```bash
soarm101 ports
soarm101 diagnose --port /dev/ttyACM0 --robot-id forge-arm
soarm101 read --port /dev/ttyACM0 --robot-id forge-arm
soarm101 configure --port /dev/ttyACM0 --robot-id forge-arm
soarm101 calibrate --port /dev/ttyACM0 --robot-id forge-arm
```

`diagnose`, `read`, and normal SDK connections do not rewrite motor configuration or enable torque.

## Kinematics validation

Record physically measured TCP checkpoints before trusting larger Cartesian moves:

```bash
soarm101 kinematics-check \
  --port /dev/ttyACM0 \
  --robot-id forge-arm \
  --sample home \
  --x-mm 391.4 --y-mm 0 --z-mm 226.5 \
  --output kinematics-validation.jsonl
```

See [docs/validation.md](docs/validation.md).

## Safety model

- Torque enable first latches every measured servo position as its goal.
- Targets are checked against model and calibrated limits.
- Paths are checked for speed, acceleration, command step, gross floor/base hazards, reach, and coarse self-clearance.
- Active motion monitors following error, unexpected direction, motor status, and timing.
- Blocking and nonblocking moves share cancellation behavior.
- `wait=True` verifies measured completion.
- `stop()` and `software_stop()` are software holds, not a physical emergency stop.

## Agentic Forge integration

Robo Studio and Robo Puppeteer wrap this library and use the shared `motion-platform:soarm101` resource identity so Robo Director can prevent conflicting commands. See [INTEGRATION.md](INTEGRATION.md).

More detail: [hardware](docs/hardware.md), [kinematics](docs/kinematics.md), [simulation](docs/simulation.md), [validation](docs/validation.md), and [safety](docs/safety.md).
