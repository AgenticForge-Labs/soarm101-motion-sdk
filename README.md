# SO-ARM101 Motion SDK

**Learn, teach, program, and automate a SO-ARM101 robot arm with Python.**

The SO-ARM101 Motion SDK is a Python-first toolkit for low-cost desktop robotics. It combines a shared motion-control library, command-line tools, a PySide6 desktop GUI, local USB-camera support, simulation, and constrained interfaces for higher-level agents.

**Connect → calibrate → move → teach → program → automate.**

![SO-ARM101 Motion SDK Setup tab showing arm connections and mechanical-stop calibration](docs/Screenshot%20From%202026-09-24%2017-34-28.png)

> **Hardware safety:** This is experimental software for an educational/hobby arm, not a safety-rated industrial controller. Never rely on software STOP/HOLD as an emergency stop. Keep a physical power cutoff within reach, start without a payload, and supervise initial motion. Read [Safety](docs/safety.md) and the [physical-run procedure](docs/physical-run.md) before moving hardware.

## SDK independence and Puppeteer MCP migration

The SO-ARM101 Motion SDK is independently useful through Python, CLI,
GUI, simulation and its authenticated trusted broker; **Puppeteer is not
required**. We are consolidating robot-to-MCP registration and agent-facing
MCP guidance into the standalone
[Forge Puppeteer](https://github.com/AgenticForge-Labs/forge-puppeteer)
project, where users can choose a lightweight MCP-only mode without
Platform or Studio.

**Source cleanup in progress:** Puppet-owned MCP implementation is merged
in Forge Puppeteer PRs #4–#6. This SDK cleanup branch removes the old
`soarm101-mcp` entrypoint and delegates optional OpenShell MCP
client packaging to Puppeteer. **Local installed-wheel/OpenShell
acceptance is not yet complete.** The native SDK and its trusted
broker remain independently usable; do not deploy this cleanup
until the cross-repository canaries pass. See
[the migration record](docs/puppeteer-mcp-migration.md).

## NVIDIA OpenShell and agentic robotics

The Motion SDK integrates with [NVIDIA OpenShell](https://github.com/NVIDIA/OpenShell), an open-source runtime that isolates AI agents and governs their filesystem and network access. This lets developers explore **agent-driven robot workflows** without handing a language model raw servo access or unrestricted control of the host workstation. OpenShell handles sandbox isolation; the Motion SDK provides the trusted, robot-specific execution boundary.

```text
Human operator: configures policy, approves capabilities, arms when appropriate
                 |
                 v
Hermes / Codex / other supported agent (reasoning, planning, camera interpretation)
                 |
                 v
NVIDIA OpenShell sandbox (isolated agent, scoped files and network)
                 |
        robotctl or Puppeteer MCP client (optional)
                 |
                 v
Authenticated host broker (pinned capabilities, lease, audit, lifecycle)
                 |
         bounded agent CLI on the physical path
                 |
                 v
SO-ARM101 Motion SDK (calibration, trajectories, rate and fault checks)
                 |
                 v
SO-ARM101 robot + named USB cameras
```

An agent can inspect live state and allowed camera observations, decide which *approved* saved pose or bounded motion to request, observe the result, and iterate. Manual and agent-driven tasks ultimately use the same deterministic SDK operations. The broker does not expose arbitrary motor register writes, raw serial devices, or an unrestricted Python interpreter on the robot host.

The SDK packages **Hermes and Codex sandbox adapters** and `robotctl` (MCP is supplied by optional Forge Puppeteer), human-pinned capability profiles, and a self-contained `soarm101 agent sandbox` workflow. Agent tool discovery does not create authority: physical motion requires a separate operator-issued lease and all motion requests remain subject to SDK validation. Host-supervisor STOP is executed outside the agent at teardown; it is not a safety-rated emergency stop.

**Current boundary:** OpenShell supports persistent native-SDK simulation and an explicit **experimental, attended physical joint mode** (`--sdk-physical`). The physical mode uses the workstation follower, a pinned profile, existing human authority and torque HOLD, separate agent/supervisor tokens, and a broker-owned heartbeat watchdog. Joint steps are capped at 2° (the example profile uses 1°), speed at 8°/s and acceleration at 25°/s². Saved poses, gripper and Cartesian actions remain outside this experimental mode. The default physical broker remains CLI-backed. Hardware watchdog qualification and production cutover remain outstanding.

See [Agent sandbox](docs/agent-sandbox.md), [MCP](docs/agent-mcp.md), and [Agent broker](docs/agent-broker.md) for supported modes, setup and safety restrictions.

## What you can do

- **Set up and calibrate the arm.** Discover serial devices, inspect motor status, confirm leader/follower assignment, and guide mechanical-stop calibration. Saved calibrations have identities used to guard motion replay.
- **Control joints and tool position.** Read measured joint state, use forward/inverse kinematics, plan joint-space and Cartesian movements, and operate the stock gripper through guarded SDK operations.
- **Teach and repeat tasks.** Save named positions, record and edit trajectories, and build deterministic programs with Move, gripper, and Wait steps. You do not need an AI agent to automate a repeatable task.
- **Use a desktop GUI.** Work with Setup, Manual control, Teleoperation, Teach/Record, recorded-motion editing, and Programs. A persistent follower sidebar displays measured state and STOP/HOLD controls.
- **Observe with USB cameras.** Configure named cameras such as `overhead` and `wrist`, preview them in the GUI, and capture still frames from the CLI and constrained agent interfaces.
- **Develop in Python or from the terminal.** The GUI, CLI, and Python SDK share motion, calibration, and saved-artifact foundations rather than implementing separate controllers.
- **Experiment with constrained agents.** An authenticated broker, bounded CLI, optional MCP adapter, and OpenShell sandbox integrations support agent workflows without granting unrestricted serial/motor access.

The project is intentionally lighter than a full ROS/MoveIt stack. It focuses on direct, understandable motion programming, calibration, teleoperation, repeatable bench workflows, and a deterministic safety boundary beneath higher-level reasoning.

## Quick start

### Requirements and installation

- Python **3.10 or newer**
- A compatible SO-ARM101 and Feetech servo interface for physical operation
- A supported computer with the relevant serial-device permissions
- No hardware required for GUI simulation or software-only tests

Clone and install the SDK with its GUI:

```bash
git clone https://github.com/AgenticForge-Labs/soarm101-motion-sdk.git
cd soarm101-motion-sdk
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
pip install -e ".[gui]"
```

Other installation options:

```bash
pip install -e .                    # SDK and CLI only
pip install -e ".[camera]"          # CLI camera capture
pip install -e ".[simulation]"      # optional PyBullet visualization
pip install -e ".[gui,simulation]"  # GUI and simulation
pip install -e ".[dev]"             # development/test dependencies
```

### Explore without hardware

```bash
soarm101-gui --simulation
soarm101 motion-envelope --json
soarm101 sdk-capabilities --json
```

To open the desktop app for real-hardware setup:

```bash
soarm101-gui
```

Use **Find Arms** to inspect detected devices before connecting. Discovery uses measured servo-bus voltage as a *hint*, not a definitive identification: a typical leader is around 5 V and follower around 12 V. Always confirm the physical ports and arm roles yourself.

### Set up a physical arm

Follow the [hardware setup guide](docs/hardware.md), [safety instructions](docs/safety.md), and [physical-run procedure](docs/physical-run.md). With the arm assembled and the correct port identified:

```bash
soarm101-setup --robot-id so101 --port /dev/ttyACM0
```

The setup utility checks the six-servo bus, applies supported motor configuration, and loads or guides calibration without enabling torque. Replace the port with your actual device (for example `COM7` on Windows).

For first powered movement, use the documented **attended, unloaded, single-joint smoke test** after confirming clearances and access to power:

```bash
soarm101 smoke-test --port /dev/ttyACM0 --robot-id so101 --joint shoulder_pan
```

**Do not operate the same serial port from the GUI and CLI simultaneously.** Calibration, pose libraries, and trajectories must match the active hardware calibration before physical replay.

## Everyday workflows

### Desktop GUI

Run `soarm101-gui` and use the tabs to progress from guided setup and manual joint/Cartesian motion to leader–follower teleoperation, taught poses, trajectories, and reusable programs.

The sidebar shows live follower joint/TCP estimates, gripper and torque state, and always-accessible hold/stop controls. Visual model geometry is an aid to interpretation, **not** proof of physical clearance or collision avoidance. See [Teleoperation](docs/teleoperation.md), [Programs](docs/programs.md), and [Kinematics](docs/kinematics.md).

### Command-line examples

Read-only inspection:

```bash
soarm101 discover
soarm101 workstation show --json
soarm101 diagnose --port /dev/ttyACM0 --robot-id so101
soarm101 read --port /dev/ttyACM0 --robot-id so101 --json
soarm101 motion-envelope --json
```

After calibration and a successful physical smoke test, a **human operator** can use guarded motion and taught positions:

```bash
soarm101 pose capture home --port /dev/ttyACM0 --robot-id so101
soarm101 pose go home --port /dev/ttyACM0 --robot-id so101
soarm101 sleep --port /dev/ttyACM0 --robot-id so101
soarm101 relax --port /dev/ttyACM0 --robot-id so101
```

Motion commands can enable torque or move the arm. `sleep` also closes the stock gripper by default; remove objects from its grasp and confirm the whole swept path is clear. Use `soarm101 --help` and `soarm101 <command> --help` for syntax and options.

### Python API

The Python SDK exposes the same calibrated motion foundation:

```python
from soarm101_motion import Pose, SOARM101

# Example for a supervised, calibrated physical arm.
with SOARM101(port="/dev/ttyACM0", robot_id="so101") as arm:
    arm.enable()
    arm.move_joints([0.0, -0.4, 0.7, 0.0, 0.2])
    current = arm.get_position()
    target = Pose(current.position + [0.01, 0.0, 0.0], current.rotation)
    arm.move_linear(target, orientation_mode="position_only", speed=0.01)
    arm.relax()
```

This is an API illustration, **not** a first-motion procedure or a guarantee of clearance for those coordinates. Follow the hardware guides and validate your own calibrated workspace before issuing a command.

### Cameras

The SDK maintains a shared workstation profile for named USB cameras:

```bash
soarm101 camera list --json
soarm101 camera capture --name overhead --json
soarm101 camera capture --all --json
```

Configure camera names/devices through the GUI or `soarm101 camera configure`. The SDK handles local camera capture; higher-level perception, camera-to-robot extrinsic calibration, and scene reasoning belong to applications above it. See [Camera](docs/camera.md).

## Agent integration

The integration rule is deliberately simple:

```text
Human / agent intent
        ↓
Constrained CLI, broker, or MCP capability
        ↓
Calibrated SO-ARM101 Motion SDK
        ↓
Servo hardware
```

A human must explicitly authorize bounded physical motion. The SDK and trusted host—not an LLM prompt—own calibration checks, limits, capability profiles, session authority, faults, and STOP/HOLD. An agent cannot obtain arbitrary motor access merely by discovering a tool.

Available interfaces include:

- The human/developer `soarm101` CLI and restricted `soarm101 agent ...` CLI.
- A broker with operator-pinned profiles, authentication, and audit events.
- An optional **Forge Puppeteer** stdio MCP interface that exposes reviewed broker capabilities without adding a second SDK MCP server.
- An OpenShell sandbox runner with Hermes, Codex, and operator-defined adapters.

**Important deployment distinction:** The standard physical agent broker remains CLI-backed. Native SDK simulation uses `--sdk-simulation`; the separate `--sdk-physical` option enables only experimental attended joint actions under the [restricted profile and watchdog contract](docs/agent-sandbox.md#attended-native-physical-joint-agents-experimental). General physical cutover still requires supervised in-motion STOP/watchdog qualification. Acknowledged software HOLD is not certified braking.

Example of **simulation-only** Codex/Puppeteer MCP operation (install `forge-puppeteer[mcp]` in the trusted host environment first):

```bash
soarm101 agent arm --simulation --minutes 10
soarm101 agent sandbox run --agent codex --auth installed \
  --interface mcp --sdk-simulation --task TASK.md \
  --output runs/codex-sdk-sim
```

This does not operate the physical arm. See [Agent CLI](docs/agent-arm101-cli.md), [Agent sandbox](docs/agent-sandbox.md), [Broker](docs/agent-broker.md), and [MCP](docs/agent-mcp.md) for current technical contracts and operator restrictions.

## Motion limits and safety

The configured absolute host motion envelope is **100°/s and 1000°/s²** for joints/tool rotation, and **100 mm/s and 1000 mm/s²** for TCP translation. These are *engineering ceilings*, **not normal defaults, safe recommendations, or proof of loaded-arm performance**. Actual requested motions can be slower and remain subject to calibration, joint/workspace limits, following-error, effort, fault, timing, STOP/HOLD, and settling checks.

The stock arm has five pose joints plus a separate gripper. Its kinematic model estimates tool position from encoder readings; that estimate cannot replace measurements of actual workspace clearance. Motion recorded against one calibration is not silently replayable against a different calibration.

An earlier physical STOP experiment showed significant continued servo motion despite an acknowledged software response. Subsequent attended trials and additional settling checks do **not** turn the arm into a safety-rated device. Keep clear of moving links and use the physical cutoff when behavior is unexpected. Do not run physical motion tests unattended or from CI.

See [Safety](docs/safety.md), [Physical run](docs/physical-run.md), and [Validation](docs/validation.md).

## Architecture and scope

The SDK owns deterministic robot motion, basic USB camera sessions, hardware diagnostics, calibration and saved-motion provenance. It does not own ROS orchestration, perception/tracking, external stage-camera calibration, or autonomous task reasoning. LeRobot inspired portions of the hardware workflow but is optional and not imported at runtime; the SDK is independently implemented.

For design details, see [Architecture](docs/architecture.md), [Kinematics](docs/kinematics.md), and [Development plan](PLAN.md).

## Development and contributions

Contributions to setup, documentation, kinematics, motion planning, teaching/programming tools, tests, and usability are welcome. For a problem report, include the operating system, arm configuration, relevant calibration details, and sanitized logs; never publish tokens or sensitive paths.

For software-only validation, install the development extras and run:

```bash
ruff check .
python -m pytest --no-cov -q
```

The test suite has hardware-free simulation and fake-transport coverage. Passing those tests does **not** establish physical safety or authorize unattended robot motion. Hardware testing follows separate operator-controlled procedures in [TESTING.md](TESTING.md).

Open an [issue](https://github.com/AgenticForge-Labs/soarm101-motion-sdk/issues) or [pull request](https://github.com/AgenticForge-Labs/soarm101-motion-sdk/pulls), or contact [AgenticForge Labs](mailto:info@agenticforgelabs.com).

## Documentation

| Topic | Guide |
| --- | --- |
| Getting started with physical hardware | [Hardware setup](docs/hardware.md) · [Physical run](docs/physical-run.md) |
| Safety and testing | [Safety](docs/safety.md) · [TESTING.md](TESTING.md) |
| Teaching and motion | [Programs](docs/programs.md) · [Teleoperation](docs/teleoperation.md) · [Kinematics](docs/kinematics.md) |
| Camera and simulation | [Camera](docs/camera.md) · [Simulation](docs/simulation.md) |
| Agent operation | [Agent CLI](docs/agent-arm101-cli.md) · [Sandbox](docs/agent-sandbox.md) · [Broker](docs/agent-broker.md) · [MCP](docs/agent-mcp.md) |
| Project structure and plans | [Architecture](docs/architecture.md) · [PLAN.md](PLAN.md) |

**License:** Apache 2.0. Maintained by AgenticForge Labs.

### Offline asset catalog

`soarm101_motion.asset_catalog.discover_assets` enumerates explicitly selected
pose, trajectory, and sequence libraries without connecting to a robot. See
`examples/asset_catalog.py`. Descriptors contain content revisions and calibration
provenance. A calibration match only compares the supplied identity; sequence
compatibility remains unchecked because referenced steps must also be validated.
Discovery does not grant replay, check live authority, or certify physical safety.


Explicitly granted recording and ordered sequence replay are available only in
supervised native SDK simulation, using operator-pinned hashes, calibration,
start tolerances, broker profile grants, watchdog and STOP fencing. Simulation
replay remains unarmed and does not create or require physical authority. Replay
reuses the broker's existing SDK session and retains request identities for
response reconciliation. Sequence steps include saved poses, recordings, gripper
targets and waits. See
[broker replay](docs/agent-broker.md#supervised-recording-replay-simulation-only).
Automatic entry motion and physical replay remain unavailable.
