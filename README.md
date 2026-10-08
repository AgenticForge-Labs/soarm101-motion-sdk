# SO-ARM101 Motion SDK

**A Python-first way to learn, teach, program, and automate the SO-ARM101.**

Low-cost arms make robotics hardware much more accessible, but there is still a gap between assembling an arm and programming it to do useful work. The SO-ARM101 Motion SDK is intended to fill that gap with a lighter, fundamentals-first environment built around a shared Python motion system, USB-camera observation layer, desktop GUI, and CLI.

The project follows a simple progression:

**connect → calibrate → move → teach → program → automate**

Continuous trajectory recording/editing remains a first-class capability: demonstrations can be replayed directly, edited into reusable motion, and later supplied as structured motion data to Robo Puppeteer. Simple automation does not require recording a motion first, so Programs and trajectory recording are complementary workflows rather than replacements for one another.

You can begin visually, learn how the robot actually moves, teach useful positions and trajectories, and then reuse those same capabilities from Python, scripts, services, or higher-level agents. Recording supports deterministic replay/editing today and is also the intended demonstration-data path toward Robo Puppeteer and later robot-learning workflows.

![SO-ARM101 Motion SDK Setup tab, showing arm connections and live mechanical-stop calibration](docs/Screenshot%20From%202026-09-24%2017-34-28.png)

## What makes it different

- **Learn robotics without hiding the robotics.** The GUI separates Setup, Manual motion, Teleoperation, Teach / Record, trajectory editing, and Programs so concepts such as calibration, joint coordinates, tool position, trajectories, and deterministic programming stay visible rather than being buried behind a single automation button.
- **Teach positions, then build a program.** Save as many named positions as needed, such as `above_pick`, `pick`, `drop`, or `camera_left`, then arrange them into a simple top-to-bottom program with Move, Open/Close gripper, custom gripper, and Wait actions. Each Move can have its own speed multiplier. A radial/pan pattern generator can expand one saved base pose into a series of moves that preserve every joint except shoulder pan. Recorded trajectories remain fully supported for replay, editing, reusable motion primitives, and future Robo Puppeteer demonstration data.
- **Go beyond servo angles.** The SDK includes forward and inverse kinematics plus Cartesian motion planning, providing a natural path from understanding individual joints to working in terms of tool position and linear movement in space. The desktop GUI now keeps one persistent right-hand robot sidebar visible across every tab. Its solid arm is always the live follower when connected, with current joints, TCP position/orientation, gripper, torque/motion/fault state, and always-available hold/stop/relax controls. Tabs add context only as ghost overlays: Teleoperation can show the leader, Teach a saved position, Edit recordings the scrubbed trajectory pose, and Programs the selected destination. The view can be rotated by dragging and reset with a double-click.
- **Calibration you can see and understand.** Guided mechanical-stop calibration visualizes two complete traversals for every joint and the gripper. Discovery verifies the SO-101 servo bus and uses measured voltage as a clue for distinguishing a typical low-voltage leader from the powered follower, while still requiring the user to confirm the hardware.
- **Safety and provenance live below the application layer.** Motion requests remain subject to calibration, joint, rate, acceleration, following-error, fault, effort, and other guards. Calibrations are versioned, and saved physical motion artifacts can be bound to the calibration under which they were created so stale motion can fail closed after a hardware or calibration change.
- **GUI, CLI, and Python share the same foundation.** The desktop application is not a separate toy controller. The interfaces reuse the same SDK operations, saved libraries, and camera settings, making it possible to start visually and transition naturally to scripting and automation.
- **Named cameras, shared hardware profile.** Follower/leader addressing and named USB cameras are persisted once in the workstation profile. Multiple uniquely named cameras such as `overhead` and `wrist` can stream concurrently; Camera and Teleoperation reuse those sessions, while CLI/agents address the same names. Teleoperation shows the named cameras together in a two-column live grid, with Start all / Stop all controls and a separate capture selector. GUI previews use latest-frame delivery, so stale display frames are discarded when rendering falls behind while fresh still captures remain available. Live camera cards show measured capture FPS, preview age, and coalesced-frame counts.
- **Live teleoperation motion trace.** The Teleoperation tab uses the same high-rate leader samples and measured follower positions already required for control to draw a rolling 10-second joint-angle view, with a switchable follower-minus-leader tracking-error view. The graph adds no extra servo/register polling; current/load remain on the existing effort-diagnostics and recording paths.
- **Persistent operating-status banner.** The top banner is always present and shows the current robot/teleoperation state when there is no newer notice. Errors and other important notices can be acknowledged with **Clear**; clearing only removes the transient message and immediately reveals the still-authoritative underlying state. Active robot faults, holding/moving state, and a teleoperation-delink condition are therefore not hidden or cleared by the UI.
- **Useful before AI, ready for AI.** Many automation tasks are deterministic: move to a known position, operate a tool, wait, move somewhere else, and repeat. The SDK makes those reliable capabilities useful on their own while also providing a constrained layer that higher-level AI systems can call.
- **A small platform for serious robotics concepts.** Calibration, coordinate systems, FK/IK, Cartesian motion, trajectory generation, teleoperation, motion recording, and sequence programming can all be explored on an inexpensive desktop arm. That makes the project useful for education, research prototyping, laboratories, small-business automation, and agentic robotics experiments.

The goal is not to replace ROS or MoveIt. Those ecosystems are valuable when a project needs broader middleware, distributed systems, sophisticated planning, or large sensor/robot integrations. This project is a lower-overhead path for people who want to begin with Python and the motion fundamentals, while leaving room to integrate into larger systems later.

LeRobot helped inspire this project’s approach to SO-ARM101 hardware and operation. Thank you to the LeRobot contributors and community. The approachable developer experience of the UFactory xArm SDK also helped shape the goal of making arm control easier to discover and use. This SDK is an independent implementation: LeRobot is optional and is not imported by the runtime.

## Motion envelope

The SDK separates **requested host motion** from the faster inner servo tracking profile.
The current absolute host envelope uses one memorable 100 / 1000 convention:

- joints: **100 deg/s** and **1000 deg/s^2**;
- TCP translation: **100 mm/s** and **1000 mm/s^2**;
- TCP/tool orientation: **100 deg/s** and **1000 deg/s^2**.

These are ceilings, not ordinary motion defaults. Existing default moves remain slower unless
the caller explicitly requests more. Cartesian IK-generated joint motion must also remain
inside the joint envelope.

Inspect the effective settings without opening hardware:

```bash
soarm101 motion-envelope
soarm101 motion-envelope --json
```

The Python API exposes the same values through `SOARM101Config.motion_limits_human` and
`SOARM101Config.from_motion_limits(...)`. CLI session commands accept matching
`--max-...-deg-s` / `--max-...-mm-s` options. `move-joints` also accepts the convenient
`--speed-deg-s` and `--acceleration-deg-s2` request units while retaining the legacy
rad/s flags.

Host-streamed planned arm motion uses the responsive Feetech tracking profile
`Goal_Velocity=0`, `Acceleration=254`. That is inner-loop tracking authority, not a
1000+ deg/s^2 host trajectory request. Calibration, following-error, current/load, hardware
fault, joint-limit, workspace, timing, STOP/HOLD, and settle checks remain active. The
100/1000 envelope is an engineering policy under physical characterization, not a claim that
a loaded arm is safe at the STS3215 no-load actuator limit.

## Programming model

The important separation is between **reasoning about what should happen** and the deterministic robot layer responsible for deciding whether and how motion can happen.

```text
person             -> GUI               -> SDK -> hardware
script             -> Python SDK        -> hardware
automation service -> SDK               -> hardware
agent              -> constrained tools -> SDK -> hardware
agent              -> camera capture    -> fresh observation
```

For agentic robotics, the intended architecture is:

```text
AI reasoning -> constrained robot capabilities -> motion SDK -> hardware
```

An agent can choose a validated capability such as moving to a taught point or executing a known sequence, but it should not need unrestricted raw motor access. It can acquire fresh camera frames through the same SDK/CLI surface, reason about them externally, and then request another constrained motion. The SDK remains responsible for motion validation, calibration context, hardware safety checks, and deterministic camera acquisition; perception and task reasoning remain above the SDK.

### Agent-facing CLI

The unrestricted `soarm101` CLI remains the human/developer interface. External reasoning
agents should instead use the bounded `soarm101 agent ...` surface. A human first runs
`soarm101 agent arm` from an interactive terminal; this parks the follower and creates a
time-limited authority lease bound to the active robot and calibration. The agent cannot
noninteractively create that authority.

The bounded surface exposes read-only state, `agent_*` saved poses, named `overhead` and
`wrist` camera capture, calibration-inset gripper open/close, Sleep, STOP/HOLD, and
translation-only Cartesian jogs. Jog policy uses the measured workspace transform only as
safety evidence: above 100 mm physical height a command may request at most 50 mm of physical
displacement; at or below 100 mm the request limit is 10 mm; targets must remain at least
10 mm above the calibrated ground plane. These are conservative command-space bounds rather
than metrology guarantees; normal SDK motion guards remain authoritative.

[`docs/agent-arm101-cli.md`](docs/agent-arm101-cli.md) documents the precise technical
contract. [`cli_use.md`](cli_use.md) is the compact operational reference. The direct-host
task-facing robot/camera skill remains under
[`agent-as-code/skills/robot-camera/SKILL.md`](agent-as-code/skills/robot-camera/SKILL.md).
The isolated OpenShell runtime uses packaged harness-specific skills for the built-in Hermes
and Codex adapters while keeping the robot contract itself harness-neutral. Machine-local
ports, calibration references, and named cameras still come from the shared workstation
profile.

The SDK includes one canonical self-contained OpenShell path for agent operation.
[docs/agent-sandbox.md](docs/agent-sandbox.md) documents
`soarm101 agent sandbox doctor/setup/run`, including a hard `run --read-only` mode whose
broker policy exposes state/camera observation but no motion routes. OpenShell supplies
process/filesystem/network isolation; the SDK owns broker lifecycle, standalone `robotctl`,
and the adapter contract. Codex can use either an OpenAI Platform API key or the existing
file-backed login from the user's installed Codex CLI; `--auth installed` copies only
`${CODEX_HOME:-~/.codex}/auth.json` into the disposable sandbox for that run and never
mounts or modifies the host Codex home. An optional separate SDK-owned ChatGPT device-login
mode is also available. Other OpenShell-compatible CLI harnesses can use the same runtime
through a validated operator-authored JSON adapter manifest; the manifest selects harness
image/provider/direct argv only and cannot broaden robot routes or authority. See
[agent-as-code/openshell-adapter.example.json](agent-as-code/openshell-adapter.example.json).
The unrestricted SDK, serial/camera devices, calibration files, Docker socket, SSH material,
and unrelated host files remain outside the reasoning sandbox. The narrow transport is
documented separately in [docs/agent-broker.md](docs/agent-broker.md). An optional **stdio MCP adapter** exposes exactly the same bounded broker routes to MCP-capable agents; see [docs/agent-mcp.md](docs/agent-mcp.md). It does not add robot authority or new motion methods. The built-in Hermes and Codex OpenShell runners may opt into it with `soarm101 agent sandbox run --interface mcp`; `robotctl` is retained as the default baseline. A trusted operator can also select a versioned `soarm101-broker --profile FILE.json` allowlist to vary which existing MCP tools, cameras, and tighter jog bounds an agent can use; that profile is enforced at the broker, not only in MCP discovery.

For attended experimental manipulation, the
[`mcp-profile-supervised-manipulation.json`](docs/examples/mcp-profile-supervised-manipulation.json)
profile offers both cameras, saved poses/Sleep, bounded joint rotations,
Cartesian moves and gripper open/close. The existing deterministic bounded
agent CLI caps physical displacement at 50 mm above 100 mm calibrated
starting TCP height and 10 mm at/below that height, independent of the
model-frame broker profile. These caps are not motion speed settings.
Review [docs/agent-mcp.md](docs/agent-mcp.md) and investigate motion
rejections/height mismatch before faster or close-object physical trials.

For direct-host Codex sessions, the MCP server now advertises an MCP-first live
robot operating preference through initialization instructions and tool
descriptions. Repository `AGENTS.md` reinforces that preference: use
`soarm101.capture_camera` for fresh overhead/wrist frames rather than
probing local camera devices or falling back to the host camera CLI.
MCP host adherence is not guaranteed, and neither guidance nor tool discovery
bypasses broker profile restrictions or human arming.

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

The `gui` extra installs PySide6 plus the camera capture dependency so live preview works out of the box. If you only need the motion SDK and CLI, use `pip install -e .` instead. For CLI camera capture without the desktop GUI, install `pip install -e ".[camera]"`. Optional visual simulation:

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

After the basic joint-direction and gripper checks pass, the paper workflows are
**measurement/calibration only**. `examples/paper_corner_cartesian_test.py` captures
three manually pointed US Letter corners with torque off, reports FK geometry, and
predicts the fourth corner in model coordinates, but it does not command Cartesian
motion.

`examples/paper_workspace_calibration.py` performs the machine-local paper/workspace
calibration. It teaches A/B/C/D on the table plus one measured D_UP reference. Those
observations persist a physical-paper -> model mapping under
`~/.config/soarm101/workspace/`; SDK/model +Z is not assumed to be physical up.

Replay software-levels A_UP/B_UP/C_UP/D_UP/CENTER_UP to the single calibrated reference
workspace Z while preserving calibrated workspace X/Y. CENTER_UP is the true calibrated
paper midpoint `(width/2, height/2, reference_height)`. A separately preflighted
calibrated-workspace-Z startup clearance is performed before lateral travel and must show
the configured minimum measured rise.

The elevated sequence is a **Cartesian linear-motion validation**:
`A_UP -> B_UP -> C_UP -> D_UP -> CENTER_UP` is executed with `move_linear()` and
position-only IK at the paper workflow's 20 Hz host cadence. The leveled model endpoints
come from one affine workspace transform, so a model-space straight line between two
equal-workspace-Z endpoints maps back to a straight constant-height line in the calibrated
workspace.

Planned Cartesian motion uses the same responsive Feetech servo-side tracking profile as
smooth live teleoperation: the host trajectory owns velocity/acceleration shaping while
the servo receives `speed_raw=0` (maximum tracking authority) and
`acceleration_raw=254`. The host path uses symmetric half-cosine acceleration/
deceleration with optional constant-speed cruise. Position-only Cartesian paths then get
one deterministic smooth-seed IK reprojection pass: joint-space seeds are filtered, every
interior Cartesian sample is re-solved at the unchanged hard position tolerance, and the
refined path is used only when it lowers discrete joint jerk. If forward sequential IK
hits a numerical pocket, a reachable endpoint solution can seed a reverse solve of those
same Cartesian samples; the paper replay reuses its exact read-only endpoint-preflight
solution for that boundary condition. The reverse path must reconnect continuously to the
measured start and still satisfy the unchanged tolerance everywhere. Because reachable
endpoints do not guarantee a reachable straight line, the paper workflow also preflights
every complete elevated segment read-only before it offers powered traversal. This keeps
the Cartesian line authoritative while reducing redundant-joint numerical wander. The previous
per-sample servo speed throttling remains superseded because it could produce visible
stick-slip on gravity-loaded joints. The public managed Cartesian controller continues to
use the fixed teleoperation profile; the #74 shake therefore remains a separate unresolved
trajectory/IK/tracking problem rather than evidence for another servo-profile change. The normal joint, IK,
rate/acceleration, following-error, fault, effort/contact, communication, settle,
provenance, and timing guards remain active. The measured workspace owns paper height;
the generic model workspace is used only where explicitly documented as a secondary
sanity check.

`--replay` reuses the saved teaching from an ordinary resting pose, and
`--measure-only` skips powered motion. The paper workflow keeps its local 3.0°/8 s
settle criterion for this qualitative hardware validation; SDK-wide defaults remain
stricter. The historical `paper_four_corner_hover_demo.py` filename remains only as a
compatibility wrapper.

Hardware testing found a case where a numerically valid +Z hover moved laterally and
contacted the table, so paper-derived normals no longer authorize powered Cartesian
motion by themselves. See [Physical kinematics validation](docs/validation.md) and
[Workspace calibration](docs/workspace-calibration.md).

Successful GUI arm connections and saved camera profiles are remembered in the shared
[machine-local workstation profile](docs/workstation.md), so the GUI, CLI, and external agents
use the same follower/leader ports, calibration references, and camera names on the next run.

## Work with the arm

The GUI keeps common tasks separate so you can start with one step and add complexity as you go:

| Tab | What you can do |
| --- | --- |
| **Setup** | Find and connect the leader and follower, persist their local ports/calibration identities, calibrate either arm, save Home and Rest, and inspect motor status. |
| **Camera** | Create named cameras such as overhead/wrist, choose stable discovered USB devices, start one or all cameras, and watch compact split/grid previews with transient-frame retry plus bounded automatic USB reconnect. |
| **Manual** | Read current joint positions, switch between angular and Cartesian arm control, keep the gripper tool visible in either mode, park/release the leader, and hand poses between leader and follower. |
| **Teleoperation** | Move the leader by hand, align/relink the follower, choose Slow/Medium/Fast tracking response independently of stream rate, choose any named live camera view, capture a picture, and transfer to Manual while parking the leader. |
| **Teach / Record** | Save named positions from the follower or leader, or record continuous demonstration trajectories for replay/editing and future Robo Puppeteer motion data. |
| **Edit recordings** | Review and adjust recorded motions with a scrubbed kinematic preview. |
| **Programs** | Build and run linear programs from saved positions, gripper actions, waits, and optional recorded-motion steps. |
| **Log** | Review connection, motion, and diagnostic events. |

The follower has five pose joints; the stock gripper is a separate tool actuator. The SDK includes forward kinematics to estimate the tool pose from joint readings, inverse kinematics for finding joint targets, and Cartesian motion planning for linear moves. These calculations use the arm model and calibration, so check the TCP, joint directions, and coordinate frame against your own assembly before relying on Cartesian accuracy. The Manual workspace exposes this explicitly: its joint markers and TCP use the SDK FK model, while visible geometry is presentation-only and independent of the coarse safety polyline. The upper and lower printed members use the local offsets from the official mesh-based SO-101 model rather than being forced through the joint centers. Each STS3215 is shown as an oriented case envelope at the official mesh pose, so a joint pivot can sit inside a motor body while the adjoining printed member ends beside it instead of at it. Small markers still show the exact kinematic joint pivots. The gripper is rendered from the modeled wrist/gripper-link frame; its moving jaw uses a neutral dark, slightly heavier stroke, and the gripper controls remain visible below both angular and Cartesian modes. Each Cartesian jog reports the requested and achieved TCP so physical/model direction mismatches can be diagnosed rather than hidden.

The GUI's gripper speed preset is shared across Manual moves, Home/Rest moves that include the gripper, teleoperation alignment/live mirroring, sequence gripper steps, and recorded-trajectory replay. Changing the preset changes subsequent commands; it does not rewrite stored trajectory timing or calibration. Teleoperation separately exposes **Tracking response**: Slow uses 0.6 rad/s and 3.0 rad/s², Medium is the previous/current behavior at 1.2 rad/s and 6.0 rad/s² and remains the default, and Fast is wrist-aware: all five joints may use the configured 100 deg/s speed ceiling, the four non-wrist-flex joints may use 1000 deg/s², and `wrist_flex` is capped at 500 deg/s² after hardware testing showed that 1000 deg/s² reversals outran the wrist's braking response. Tracking response limits how quickly the follower closes the leader gap; stream rate remains a separate control. The same per-joint ceilings are enforced by the deterministic stream guard, and the configured absolute motion envelope remains the hard upper bound. Live streaming also permits only a short, non-growing carry-through immediately after a commanded reversal, capped at 0.10 rad total wrong-way travel; ordinary planned motion retains the strict opposite-direction rule. A separate **Sleep** operation is derived from each follower's calibration, providing a stable folded posture for demos and power-down preparation without replacing user-saved Home/Rest. The default Sleep keeps shoulder pan at midpoint, shoulder lift at its lower executable limit, elbow flex at its upper executable limit, wrist roll at midpoint, and places **wrist flex 75% of the way from its lower executable limit to its upper limit** (`upper - 0.25 * (upper - lower)`). Physical testing showed this less wrist-up geometry settles substantially more smoothly than the historical fully folded posture. That original posture remains available as **sleep_up** / `move_sleep_up()`, with wrist flex at its lower executable limit. Both postures close the stock gripper to a target **1° inside its calibrated closed mechanical stop** by default. Sleep does not know whether the gripper is holding an object: a pen held during Sleep triggered the servo's overload protection, so remove held objects or otherwise account for the closing action before invoking Sleep. Because the shoulder/elbow fold is closer than the generic coarse 25 mm link-centerline self-clearance heuristic, these dedicated Sleep-family primitives skip only that coarse arm workspace-geometry check; calibrated joint/tool limits, trajectory, following-error, effort, fault, and completion guards remain active.

The GUI automatically keeps displayed joint readings current and provides explicit controls for editing and moving to targets. During commanded arm or gripper motion, the sidebar now consumes measured feedback from the owning motion/tool thread, so the joint schematic, model-estimated TCP readout, and moving jaw update while the normal slower GUI state poll is intentionally paused. Ordinary arm and gripper moves reuse measurements already required for safety/progress; combined recorded/alignment paths sample the gripper only at the owning controller's existing feedback checkpoints. This adds no independent or competing serial poller and has no authority over motion or safety. A resizable workspace keeps the task tabs on the left and the persistent follower sidebar on the right, including Setup and Log, so robot state never disappears while changing workflows. If vertical space is tight, the model/readout area scrolls while Enable hold, STOP/HOLD, and Relax remain pinned and visible. Programs are stored using the existing `MotionSequence` format, so GUI Programs and SDK/CLI sequence execution share the same guarded runner and provenance rules. See [Programs and saved positions](docs/programs.md) for the simple position-program workflow. Session logs are enabled by default and stored under `~/.local/state/soarm101/gui/` on Linux.

The sidebar's model schematic shows URDF-visual link-body centerlines, a fixed finger,
and a rotating stock jaw. Before connection the illustration uses a neutral 45% jaw opening
rather than the extreme-open limit; once connected it follows measured aperture. Side, Front,
Top, and Isometric views help inspect depth; drag rotates and wheel zooms. The scale remains
fixed while the robot moves. Use Fit to reframe once or enable Auto fit for continuous
reframing. A screen-plane ruler shows millimeters; the grid is model Z=0 rather than the
measured table. The TCP marker uses the session's active tool transform. Presentation
geometry is intentionally separate from the coarse safety centerline and does not certify
printed-housing, collision, or physical Cartesian accuracy. See PLAN.md for the remaining
measurement and shaking work.

### CLI examples

After a successful GUI follower connection has been saved in the workstation profile,
one-off session commands can omit `--port`; explicit connection arguments still override
the saved follower.

```bash
# Inspect the saved workstation and current follower state
soarm101 workstation show --json
soarm101 read --json

# Find arms and inspect their measured voltage and motor status
soarm101 discover
soarm101 diagnose --port /dev/ttyACM0 --robot-id so101

# Read calibrated joints/TCP or solve a target without moving
soarm101 read --port /dev/ttyACM0 --robot-id so101 --json
soarm101 ik --port /dev/ttyACM0 --robot-id so101 \
  --x-mm 180 --y-mm 0 --z-mm 160 --orientation-mode position_only --json

# Move the five pose joints (degrees)
soarm101 move-joints --port /dev/ttyACM0 --robot-id so101 \
  --degrees 0 -20 35 0 10 --yes

# Capture and replay a named pose
soarm101 pose capture home --port /dev/ttyACM0 --robot-id so101
soarm101 pose go home --port /dev/ttyACM0 --robot-id so101 --yes
# Successful pose replay remains torque-held; release only with explicit ENTER confirmation
soarm101 relax --port /dev/ttyACM0 --robot-id so101

# Move to the smoother calibration-derived default Sleep posture
soarm101 sleep --port /dev/ttyACM0 --robot-id so101 --yes
# Historical fully folded wrist-up posture
soarm101 sleep-up --port /dev/ttyACM0 --robot-id so101 --yes

# Operate the stock gripper
soarm101 gripper --port /dev/ttyACM0 --robot-id so101 open --yes

# Inspect the machine-local hardware profile and named cameras
soarm101 workstation show --json
soarm101 camera list --json
soarm101 camera configure --name overhead --device /dev/v4l/by-id/CAMERA-video-index0 \
  --width 1280 --height 720 --fps 30 --fourcc MJPG --json
soarm101 camera capture --name overhead --json
soarm101 camera capture --all --json
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

This is experimental software for a low-cost educational and hobby arm, not a certified industrial controller. Start without a payload, clear the work area, keep physical power accessible, and verify calibration and directions on your own assembly. Before motion, make sure the selected ports refer to the intended leader and follower, check their voltage and servo status, and confirm the saved calibration matches each arm. Begin with small movements and no payload. On torque enable, a relaxed measured position no more than 8 encoder ticks (about 0.7°) beyond an active EEPROM limit is clamped inward only for the startup latch; larger violations still block torque enable, and commanded motion remains inside the calibrated limits. Software stop holds the current position as best it can; it cannot replace the physical power switch or an emergency stop. Current/load readings help detect effort changes but are not calibrated force measurements. See [docs/safety.md](docs/safety.md), [TESTING.md](TESTING.md), and the [physical-run procedure](docs/physical-run.md).

If an existing follower was configured by an earlier SDK version that used a position
P gain of 16, run `soarm101 configure --port <PORT> --robot-id <ID>` once after updating.
The current recommended STS3215 position P gain is the factory value 32; this configuration
step does not redo mechanical-stop calibration.

Simulation and fake-transport tests cover the motion and hardware interfaces. Physical behavior depends on the specific arm, assembly, calibration, power supply, and payload; test cautiously before relying on a movement or saved trajectory. Leader parking and cross-arm pose matching enable torque and can move a physical arm; treat them as powered-motion operations even though the leader is normally back-drivable with torque off.

Motion-quality debugging can use `scripts/run_motion_quality_study.sh` for the guided teleop-versus-programmed experiment. The runner updates the active diagnostic branch, launches the GUI, marks and extracts the operator's slow teleop reference interval, optionally replays the exact accepted teleop arm-joint commands through guarded streaming, then runs the same saved-pose Sleep/Overhead/Left/Right route automatically at 50 Hz and 20 Hz. It packages the complete evidence under `~/soarm-motion-tests/` for review. `examples/motion_quality_trace.py` remains the lower-level automatic programmed-route tracer. Passive traces record existing command writes, natural feedback reads, raw encoder targets, TCP positions, effective servo speed/acceleration parameters, hardware-state checks, and route markers without adding motion-time hardware polling. Detailed GUI teleoperation logging is enabled by default and writes `teleop_frame` evidence under `~/.local/state/soarm101/gui/`. If the in-run exact replay is skipped because the follower is not close enough to the first captured command, finish the study and run `scripts/run_motion_quality_replay.sh`. The post-study runner selects the latest study by default, uses the normal guarded joint-motion primitive to pre-position to the first recorded arm pose, verifies measured arrival, preflights every recorded stream sample against the active joint/step/speed/acceleration limits, then replays and appends its trace/summary back into the same study archive.

Cartesian `move_linear()` trajectories are parameterized in Cartesian space with
half-cosine acceleration/deceleration and optional cruise, then solved by sequential IK
at the host command rate. Position-only paths may then reproject from smoothed joint seeds
at the same Cartesian tolerance, reducing redundant-joint jitter without altering the
requested line.
On calibrated hardware, planned motion also suppresses intermediate host writes whose five
joint targets quantize to exactly the same encoder counts as the last command. The original
host deadlines, safety monitoring, and exact final planned sample are preserved; this is
command de-duplication at actuator resolution, not trajectory retiming or relaxed dynamics.

Joint-space motion now also has an **experimental** execution selector while the low-speed
shake investigation is active. `execution_mode="streamed"` remains the default and sends
the validated host trajectory as intermediate targets. Streamed planned joint motion now
uses the same responsive Feetech tracking profile as teleoperation—Goal_Velocity=0
(unrestricted/max) and acceleration=254—so the validated host trajectory owns the requested
speed and acceleration instead of being overlaid with the backend's slower 250/20 profile.
Joint/rate/acceleration/following-error/fault/effort/timing/settle guards remain active.
`execution_mode="final_target"`
builds and validates the same joint plan but writes only the endpoint once, using per-joint
servo speed limits derived from the validated planned duration so the joints are asked to
arrive together. The controller then observes guarded progress until settle. The one-shot
mode retains calibrated limits, validated path/workspace policy, fault handling, cancellation,
unexpected-direction checks, a per-joint start-to-target corridor/overshoot bound, timeout,
and final settle; it does not apply endpoint following error or cross-joint phase matching
during the expected transit because the servos are intentionally executing the endpoint
internally and may progress at different rates under load. For ordinary full-workspace and
saved-pose moves, each measured intermediate configuration is rechecked with the same
measured-start workspace validator, so asynchronous servo progress is not assumed to follow
the synchronized host plan. Explicit target-only/off workspace modes retain their existing
semantics, and Sleep retains its existing coarse-workspace exception. Cartesian `move_linear()` remains
host-streamed because a single joint endpoint cannot guarantee the requested straight TCP
path. Use `scripts/run_joint_execution_comparison.sh` for the supervised A/B route.

The SDK has a five-joint arm model, a separate stock-gripper tool, joint and Cartesian motion, forward and inverse kinematics, trajectory recording and playback, deterministic sequence programming, simulation, basic local USB-camera capture, and an optional PySide6 GUI. Higher-level perception/tracking, calibrated multi-camera stage systems, ROS integration, and show orchestration remain outside this project. The SDK is intended to provide constrained motion plus a deterministic local observation surface beneath those higher-level systems. See [Architecture](docs/architecture.md) and [Camera](docs/camera.md) for the boundaries.

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
- [Camera](docs/camera.md)
- [Validation](docs/validation.md)
- [Architecture](docs/architecture.md)
- [Development plan](PLAN.md)

### Setup for new and returning users

Setup opens on a compact arm overview. Valid saved calibration is reused; **Saved
calibration found** does not mean a device is connected. **Calibration loaded** is
shown only from the connected session's matching robot identity. Discovery suggests
arm assignments; confirm physical devices under **Manage…**. Connecting and enabling
hold remain separate explicit actions.

Choose **Set up this arm…** when calibration is missing, or **Manage… → Calibrate /
Recalibrate…** for a changed setup. Calibration opens inside Setup and keeps the
persistent follower sidebar and STOP/HOLD visible. Calibration records hand movement
with motors off. **Back to Setup** is blocked while a sweep is active; finish or cancel
it first. Existing cancellation/rollback and motion guards are unchanged.

**Guide me through setup…** opens a state-driven follower walkthrough: discovery,
explicit device confirmation, calibration or connection, then Manual. It skips
calibration when a valid profile exists and never starts motion.

**Show setup guidance** remembers your preference and controls short task instructions
across the workspace. Experienced users can hide them without losing direct controls.
**Diagnostics / advanced…** contains detailed logging and effort characterization.

**Back up setup…** exports calibration aliases/history, workspace calibration, and
workstation settings to a JSON archive outside Git. **Restore setup…** validates all
entries, requires both arms disconnected, preserves replaced files in
`~/.config/soarm101/restore-history/`, and closes the GUI so the next launch reloads
settings. Confirm ports and camera devices after moving a backup to another machine.
Backups do not include saved motion libraries or camera images. Keep a copy on a
separate disk or backup service; local calibration history alone is not a disk backup.
