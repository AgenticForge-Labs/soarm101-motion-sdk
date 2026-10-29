# Agent instructions

These instructions apply to people and coding agents developing the SO-ARM101 Motion SDK, and to agents using the approved robot interfaces. **Code development and physical robot operation are different roles.** Read the authoritative files and actual capabilities rather than old PR descriptions.

## Current integration boundary

The repository includes the NVIDIA OpenShell sandbox integration, packaged Hermes/Codex adapters, `robotctl` and optional stdio MCP clients, an authenticated host broker, operator-pinned profiles, and a deterministic calibrated Motion SDK.

- **Ordinary physical agent path:** sandboxed agent → permitted `robotctl`/MCP tool → trusted host broker → bounded `soarm101 agent ...` CLI → Motion SDK → hardware.
- **Optional native SDK simulation:** `soarm101 agent sandbox run --sdk-simulation` uses one simulated persistent SDK session behind a capability-restricted relay. The agent receives a separate token from the trusted host's private, loopback STOP controller. There is no physical serial connection or real camera access in this mode.
- **Restricted trusted-host physical trials:** operator-only experimental SDK broker scripts are not general agent authority, a released native physical broker, or proof of safety-rated stopping.
- **Experimental attended physical mode:** `sandbox run --sdk-physical` uses the configured follower, immutable narrow joint/camera profile, existing human torque HOLD and lease, and separate-token relay. The broker watchdog is implemented and software-tested; physical STOP/watchdog qualification and general production cutover remain outstanding. No automatic hardware tests, deployment, or motion are authorized by this mode. An earlier proposed milestone called “PR #94” was never that implementation; actual merged PR #94 only cleaned the README.

OpenShell governs sandbox files/network/process access; it does **not** decide what motion is safe. The trusted broker owns authorization, profile and lease admission, auditing, supervisor lifecycle and private STOP routing. The canonical SDK/controller/backend own calibration, joint/workspace/rate/effort/fault checks, motion execution and actual STOP/HOLD. Never create a competing safety or motor-controller path.

## Before development

1. Read the current `README.md`, `PLAN.md` and `docs/architecture.md`. For substantial cross-repository changes, also read organization-level `development.md` and `architecture.md`.
2. Check current `main`, open PRs, recent commits, and applicable tests. Do not treat notes about earlier draft PRs as unmerged or current without verifying GitHub.
3. Read `docs/safety.md` before physical motion and `docs/physical-run.md` before a first or changed-hardware trial.
4. Validate behavior with software-only tests and simulation before any separately operator-approved physical gate. CI and unattended coding agents must never run active robot movement.
5. When CLI/UI/behavior or safety contracts change, update README, docs, TESTING, CHANGELOG and examples as appropriate. Preserve clear states: implemented, simulation only, operator-only experimental, or future work.

## Hardware and motion invariants

- Human arming is interactive, explicit, time-limited and calibration-bound. Sandboxes and agents cannot arm themselves, extend leases, reconfigure motors, load an unrestricted physical motion profile, or bypass authority.
- The robot owns its servo connection. Do not open the same serial port from GUI, CLI and broker at once. Linux cooperative advisory locks cannot exclude legacy software that ignores them.
- Saved poses, sequences and recordings use calibration provenance. Validate identity and full paths before motion; never replay stale or mismatched calibration data.
- Do not bypass measured joint/tool limits, workspace validation, following-error, effort/fault, movement rate/acceleration, timing, state or settling checks to make a run succeed.
- The configured 100/1000 host envelope is a characterization ceiling, **not** a recommended physical speed. Operator/pinned agent rates may further restrict requests and apply to all supported joint-space actions including Sleep/saved poses.
- Real physical +Z/table height must be established from calibration and measurements, not inferred solely from model +Z. Treat printed/link presentation geometry as illustrative, not proven physical clearance.
- Servo STOP/HOLD is **not a safety-rated emergency stop**. Prior physical evidence included 18.4° continued elbow motion after an acknowledged software STOP. Later attended STOP checks are narrower evidence, not a universal hardware braking guarantee. Fail closed on incomplete or unverified HOLD, never auto-resume or auto-rearm, and keep a physical cutoff reachable.
- STOP and cleanup must attempt both arm and gripper cancellation where appropriate, reject late action success after STOP, revoke the associated authority, and preserve fault/evidence semantics. No reconnect on terminal SDK session faults.
- Keep camera acquisition and named camera profiles in this SDK, but do not mistake estimated robot coordinates or camera images for independently calibrated physical collision sensing.

## Live robot tasks with a connected MCP server

For requests to **inspect or operate the live robot**, prefer the connected `soarm101` MCP tools when they are available. Use dynamic `robot_state` / `robot_capabilities` and approved `capture_camera`, `go_pose`, `sleep` or bounded `jog_cartesian` only when advertised. The allowed tool set depends on the trusted host's profile.

Do not inspect `/dev/video*`, mount raw devices, invoke unrestricted host CLI or try alternate broker URLs as an agent workaround for a denied/missing tool. MCP prompts and resources teach semantics but cannot provide fresh state or override permissions. Inspect actual returned image pixels and report unavailable capabilities honestly.

These MCP-first preferences do not prohibit development tests, operator-led diagnostics, or the human `soarm101` CLI.

## OpenShell and broker safeguards

- The canonical NVIDIA OpenShell sandbox must not mount the Motion SDK source tree, serial or camera devices, calibration records, Docker socket, SSH keys, or unrelated host files.
- The sandbox may use packaged client-only `robotctl` or stdio MCP software. The MCP client's narrowly readable `/opt/soarm101-mcp` environment is not a robot device mount or writable host volume.
- Hermes and Codex are packaged adapters; other OpenShell-compatible harnesses may use an operator-authored validated adapter manifest. Harness configuration must never change robot authority, broker endpoint ownership, available motion operations, or API semantics.
- Operator capability profiles are immutable per run and enforced **at the broker**, not only in MCP tool discovery. Read-only mode restricts actual broker routes. Clients may ask for slower rates but cannot widen rate bounds.
- Credentials and host-private STOP tokens must not appear in agent context or committed traces. Separate agent-relay and host-supervisor tokens in native SDK simulation.
- The trusted host, not the language model, requests STOP before motion-capable sandbox teardown. Failed or unconfirmed STOP means an unsafe/unconfirmed run. Read-only runs do not issue motor STOP. Process death/power loss can preclude cleanup and is not mitigated by prompting.

See `docs/agent-sandbox.md`, `docs/agent-broker.md`, `docs/agent-mcp.md` and `docs/agent-arm101-cli.md` for exact contracts and operator commands. Keep `cli_use.md` policy-neutral and place benchmarking/scoring above this repository.

## Using the CLI / operating the arm

1. Inspect connections with `soarm101 discover`, `soarm101 diagnose`, and read-only state before enabling torque.
2. Set up/calibrate changed or new hardware through documented commands. Do not disable the calibration-provenance gate.
3. Start real movement with a supervised, unloaded, single-joint `soarm101 smoke-test`, clear workspace and reachable power cutoff.
4. Reuse the SDK's guarded primitives for Python, GUI and CLI rather than direct servo-register writes. Only perform explicit low-level hardware development under separately reviewed procedures.
5. Treat software simulation and fake transports as software evidence only. Never automatically run hardware tests from CI/Codex or on a timer.
6. For unresolved Cartesian geometry or shaking problems, start from read-only FK/IK/trajectory and workspace diagnostics in `docs/validation.md` and `TESTING.md`. Do not replace Cartesian linear tests with convenient joint replay or speculative servo PID changes.

## Repository boundary

The SDK owns SO-ARM101 calibration, kinematics, motion, tools/TCP, guards, diagnostics, saved-position/recording/program semantics, local USB cameras and the constrained robot-specific agent execution boundary. Higher-level perception, stage systems, ROS middleware, performance orchestration, and cross-robot coordination belong to other systems.
