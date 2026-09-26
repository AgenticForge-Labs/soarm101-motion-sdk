# SO-ARM101 Motion SDK

**A Python-first way to learn, teach, program, and automate the SO-ARM101.**

Low-cost arms make robotics hardware much more accessible, but there is still a gap between assembling an arm and programming it to do useful work. The SO-ARM101 Motion SDK is intended to fill that gap with a lighter, fundamentals-first environment built around a shared Python motion system, desktop GUI, and CLI.

The project follows a simple progression:

**connect → calibrate → move → teach → program → automate**

Continuous trajectory recording/editing remains a first-class capability: demonstrations can be replayed directly, edited into reusable motion, and later supplied as structured motion data to Robo Puppeteer. Simple automation does not require recording a motion first, so Programs and trajectory recording are complementary workflows rather than replacements for one another.

You can begin visually, learn how the robot actually moves, teach useful positions and trajectories, and then reuse those same capabilities from Python, scripts, services, or higher-level agents. Recording supports deterministic replay/editing today and is also the intended demonstration-data path toward Robo Puppeteer and later robot-learning workflows.

![SO-ARM101 Motion SDK Setup tab, showing arm connections and live mechanical-stop calibration](docs/Screenshot%20From%202026-09-24%2017-34-28.png)

## What makes it different

- **Learn robotics without hiding the robotics.** The GUI separates Setup, Manual motion, Teleoperation, Record / Teach, editing, and Run workflows so concepts such as calibration, joint coordinates, tool position, trajectories, and sequencing stay visible rather than being buried behind a single automation button.
- **Teach positions, then build a program.** Save as many named positions as needed, such as `above_pick`, `pick`, `drop`, or `camera_left`, then arrange them into a simple top-to-bottom program with Move, Open/Close gripper, custom gripper, and Wait actions. Each Move can have its own speed multiplier. A radial/pan pattern generator can expand one saved base pose into a series of moves that preserve every joint except shoulder pan. Recorded trajectories remain fully supported for replay, editing, reusable motion primitives, and future Robo Puppeteer demonstration data.
- **Go beyond servo angles.** The SDK includes forward and inverse kinematics plus Cartesian motion planning, providing a natural path from understanding individual joints to working in terms of tool position and linear movement in space. The desktop GUI now keeps one persistent right-hand robot sidebar visible across every tab. Its solid arm is always the live follower when connected, with current joints, TCP position/orientation, gripper, torque/motion/fault state, and always-available hold/stop/relax controls. Tabs add context only as ghost overlays: Teleoperation can show the leader, Teach a saved position, Edit recordings the scrubbed trajectory pose, and Programs the selected destination. The view can be rotated by dragging and reset with a double-click.
- **Calibration you can see and understand.** Guided mechanical-stop calibration visualizes two complete traversals for every joint and the gripper. Discovery verifies the SO-101 servo bus and uses measured voltage as a clue for distinguishing a typical low-voltage leader from the powered follower, while still requiring the user to confirm the hardware.
- **Safety and provenance live below the application layer.** Motion requests remain subject to calibration, joint, rate, acceleration, following-error, fault, effort, and other guards. Calibrations are versioned, and saved physical motion artifacts can be bound to the calibration under which they were created so stale motion can fail closed after a hardware or calibration change.
- **GUI, CLI, and Python share the same foundation.** The desktop application is not a separate toy controller. The interfaces reuse the same SDK operations and saved libraries, making it possible to start visually and transition naturally to scripting and automation.
- **Useful before AI, ready for AI.** Many automation tasks are deterministic: move to a known position, operate a tool, wait, move somewhere else, and repeat. The SDK makes those reliable capabilities useful on their own while also providing a constrained layer that higher-level AI systems can call.
- **A small platform for serious robotics concepts.** Calibration, coordinate systems, FK/IK, Cartesian motion, trajectory generation, teleoperation, motion recording, and sequence programming can all be explored on an inexpensive desktop arm. That makes the project useful for education, research prototyping, laboratories, small-business automation, and agentic robotics experiments.

The goal is not to replace ROS or MoveIt. Those ecosystems are valuable when a project needs broader middleware, distributed systems, sophisticated planning, or large sensor/robot integrations. This project is a lower-overhead path for people who want to begin with Python and the motion fundamentals, while leaving room to integrate into larger systems later.

LeRobot helped inspire this project’s approach to SO-ARM101 hardware and operation. Thank you to the LeRobot contributors and community. The approachable developer experience of the UFactory xArm SDK also helped shape the goal of making arm control easier to discover and use. This SDK is an independent implementation: LeRobot is optional and is not imported by the runtime.

## Programming model

The important separation is between **reasoning about what should happen** and the deterministic robot layer responsible for deciding whether and how motion can happen.

```text
person             -> GUI               -> SDK -> hardware
script             -> Python SDK        -> hardware
automation service -> SDK               -> hardware
agent              -> constrained tools -> SDK -> hardware
```

For agentic robotics, the intended architecture is:

```text
AI reasoning -> constrained robot capabilities -> motion SDK -> hardware
```

An agent can choose a validated capability such as moving to a taught point or executing a known sequence, but it should not need unrestricted raw motor access. The SDK remains responsible for motion validation, calibration context, and hardware safety checks.

## Get started

### Install

This is a **Python 3.10+** project. The base install provides the Python SDK and the `soarm101` / `soarm101-setup` command-line tools. Its runtime dependencies are NumPy, SciPy, PySerial, and the Feetech servo SDK. The desktop app is a separate optional GUI extra powered by PySide6; PyBullet is an optional extra for visual simulation.

Install the SDK, CLI, and GUI from source:

```bash
git clone https://github.com/AgenticForge-Labs/soarm101-motion-sdk.git
cd soarm101-motion-sdk

python3 -m venv .venv
source .venv/bin/activate       # Windows: .venv\Scripts\activate
python -m pip install --upgrade pip
pip install -e ".[gui]"
```

The `gui` extra installs PySide6. If you only need the SDK and CLI, use `pip install -e .` instead. Optional visual simulation:

```bash
pip install -e ".[simulation]"  # PyBullet visual simulation
# or install GUI and simulation together:
pip install -e ".[gui,simulation]"
```

### Open the GUI

```bash
soarm101-gui
```

You can also open it without connected hardware:

```bash
soarm101-gui --simulation
```

In the GUI, use **Find Arms** to inspect available devices. Discovery is read-only: it does not connect or enable torque. The common SO-ARM101 setup has a low-voltage leader (around 5 V) that you move by hand and a powered follower (around 12 V) that moves under control. The GUI uses measured servo-bus voltage as a role hint, then lets you confirm or change the selected ports before you press **Connect**. Voltage is a convention for this common setup, not a definitive identity or a safety check: verify which physical arm is which and confirm the correct port if voltage is unexpected, the setup differs, or more than one device fits a role. You can select ports manually in the Setup tab. Calibration and motion controls are available in their own tabs.

To set up an assembled arm from the terminal, run:

```bash
soarm101-setup --robot-id so101
```

Setup checks the six-motor bus, applies recommended motor settings, reuses a usable calibration or guides you through one, and saves the calibration. It does not enable torque. Add `--port /dev/ttyACM0` (Linux) or `--port COM7` (Windows) if the serial port is not unambiguous.

Calibrations are versioned and identified by a content-derived ID. Saved poses and motion artifacts record which calibration they were made with; physical replay stops if the active follower calibration does not match. See the [physical-run procedure](docs/physical-run.md) before your first hardware motion.

For a new arm, review the [hardware setup guide](docs/hardware.md) before connecting the motors. First powered movement should be a supervised, no-payload smoke test with physical power within reach:

```bash
soarm101 smoke-test --port /dev/ttyACM0 --robot-id so101 --joint shoulder_pan
```

Read [Safety](docs/safety.md) before moving hardware. Do not open the same serial port in the GUI and CLI at the same time.

## Work with the arm

The GUI keeps common tasks separate so you can start with one step and add complexity as you go:

| Tab | What you can do |
| --- | --- |
| **Setup** | Find and connect the leader and follower, calibrate either arm, save Home and Rest, and inspect motor status. |
| **Manual** | Read current joint positions, switch between angular and Cartesian arm control, keep the gripper tool visible in either mode, park/release the leader, and hand poses between leader and follower. |
| **Teleoperation** | Move the leader by hand, align the follower or relink the two current poses with no motion, and transfer to Manual while parking the leader. |
| **Teach / Record** | Save named positions from the follower or leader, or record continuous demonstration trajectories for replay/editing and future Robo Puppeteer motion data. |
| **Edit recordings** | Review and adjust recorded motions with a scrubbed kinematic preview. |
| **Programs** | Build and run linear programs from saved positions, gripper actions, waits, and optional recorded-motion steps. |
| **Log** | Review connection, motion, and diagnostic events. |

The follower has five pose joints; the stock gripper is a separate tool actuator. The SDK includes forward kinematics to estimate the tool pose from joint readings, inverse kinematics for finding joint targets, and Cartesian motion planning for linear moves. These calculations use the arm model and calibration, so check the TCP, joint directions, and coordinate frame against your own assembly before relying on Cartesian accuracy. The Manual workspace exposes this explicitly: its joint-center view uses the SDK FK model, renders the gripper from the modeled wrist/gripper-link frame, and keeps the gripper controls visible below both angular and Cartesian modes. Each Cartesian jog reports the requested and achieved TCP so physical/model direction mismatches can be diagnosed rather than hidden.

The GUI's gripper speed preset is shared across Manual moves, Home/Rest moves that include the gripper, teleoperation alignment/live mirroring, sequence gripper steps, and recorded-trajectory replay. Changing the preset changes subsequent commands; it does not rewrite stored trajectory timing or calibration.

The GUI automatically keeps displayed joint readings current and provides explicit controls for editing and moving to targets. A resizable workspace keeps the task tabs on the left and the persistent live follower sidebar on the right, including Setup and Log, so robot state never disappears while changing workflows. Programs are stored using the existing `MotionSequence` format, so GUI Programs and SDK/CLI sequence execution share the same guarded runner and provenance rules. See [Programs and saved positions](docs/programs.md) for the simple position-program workflow. Session logs are enabled by default and stored under `~/.local/state/soarm101/gui/` on Linux.

### CLI examples

```bash
# Find arms and inspect their measured voltage and motor status
soarm101 discover
soarm101 diagnose --port /dev/ttyACM0 --robot-id so101

# Move the five pose joints (degrees)
soarm101 move-joints --port /dev/ttyACM0 --robot-id so101 \
  --degrees 0 -20 35 0 10 --yes

# Capture and replay a named pose
soarm101 pose capture home --port /dev/ttyACM0 --robot-id so101
soarm101 pose go home --port /dev/ttyACM0 --robot-id so101 --yes

# Operate the stock gripper
soarm101 gripper --port /dev/ttyACM0 --robot-id so101 open --yes
```

Run `soarm101 --help` or `soarm101 <command> --help` for more commands and options. The Python SDK is the programmatic motion interface, and the CLI exposes setup, diagnostics, guarded motion, saved poses, trajectory playback, and sequence playback. These interfaces make the same motion capabilities available to scripts and agentic applications. The GUI currently provides the authoring workflow for recording, editing, and live leader-to-follower teleoperation; CLI coverage for those workflows is still developing.

### Python example

```python
from soarm101_motion import Pose, SOARM101

with SOARM101(port="/dev/ttyACM0", robot_id="so101") as arm:
    arm.enable()
    arm.move_joints([0.0, -0.4, 0.7, 0.0, 0.2])
    arm.tool.open()

    current = arm.get_position()
    target = Pose(current.position + [0.01, 0.0, 0.0], current.rotation)
    arm.move_linear(target, orientation_mode="position_only", speed=0.01)

    arm.relax()
```

## Safety and project status

This is experimental software for a low-cost educational and hobby arm, not a certified industrial controller. Start without a payload, clear the work area, keep physical power accessible, and verify calibration and directions on your own assembly. Before motion, make sure the selected ports refer to the intended leader and follower, check their voltage and servo status, and confirm the saved calibration matches each arm. Begin with small movements and no payload. On torque enable, a measured position no more than 8 encoder ticks (about 0.7°) beyond an active EEPROM limit is clamped inward to that limit and logged; larger violations still block torque enable. Software stop holds the current position as best it can; it cannot replace the physical power switch or an emergency stop. Current/load readings help detect effort changes but are not calibrated force measurements. See [docs/safety.md](docs/safety.md), [TESTING.md](TESTING.md), and the [physical-run procedure](docs/physical-run.md).

Simulation and fake-transport tests cover the motion and hardware interfaces. Physical behavior depends on the specific arm, assembly, calibration, power supply, and payload; test cautiously before relying on a movement or saved trajectory. Leader parking and cross-arm pose matching enable torque and can move a physical arm; treat them as powered-motion operations even though the leader is normally back-drivable with torque off.

The SDK has a five-joint arm model, a separate stock-gripper tool, joint and Cartesian motion, forward and inverse kinematics, trajectory recording and playback, deterministic sequence programming, simulation, and an optional PySide6 GUI. ROS, camera capture, tracking, and show orchestration are outside this project. The SDK is intended to remain the constrained motion layer beneath those higher-level systems. See [Architecture](docs/architecture.md) for the boundaries.

### Where we want to go

We want to keep making the interface more polished, easier to learn, and more useful at the bench. Near-term areas include clearer onboarding and status, a more comfortable calibration and teaching experience, and stronger tools for recording, editing, organizing, and programming automated workflows that can run repeatedly without AI. We also want the Python SDK and CLI to remain reliable foundations for scripts, robotics integrations, and agentic control. See [PLAN.md](PLAN.md) for current work; feedback on priorities is welcome.

## Contribute

Feedback, bug reports, pull requests, and other collaborations are welcome. If you are learning the SO-ARM101, building teaching workflows, improving kinematics or calibration, making the interface easier to use, or developing repeatable robot programs, we would like to hear from you. Please include your operating system, arm setup, calibration details, and relevant session log when reporting a problem; remove any information you do not want to share.

Open an issue or pull request on [GitHub](https://github.com/AgenticForge-Labs/soarm101-motion-sdk), or contact Agentic Forge Labs at [info@agenticforgelabs.com](mailto:info@agenticforgelabs.com) about the project or company.

## Further reading

- [Hardware setup](docs/hardware.md)
- [Safety](docs/safety.md)
- [Teleoperation](docs/teleoperation.md)
- [Kinematics](docs/kinematics.md)
- [Programs and saved positions](docs/programs.md)
- [Simulation](docs/simulation.md)
- [Validation](docs/validation.md)
- [Architecture](docs/architecture.md)
- [Development plan](PLAN.md)
