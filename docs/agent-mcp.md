# Optional MCP adapter for the bounded SO-ARM101 broker

The `soarm101-mcp` command is a **stdio MCP server**, not another robot controller,
not a new network listener, and not a replacement for the existing `robotctl` client.
It translates a fixed set of typed MCP tools into the **existing authenticated**
`soarm101-broker` HTTP/JSON allowlist. Every physical command remains subject to
the broker's trusted-host motion envelope, interactive human arming/calibration lease,
and Motion SDK's deterministic safety rules.

MCP is optional: `pip install -e '.[mcp]'`. Base SDK, CLI, broker, and GUI
installations require no MCP dependency.

## Start (local, read-only validation)

Run on a workstation with the Motion SDK installed, and with the robot **not armed**.
Set a randomly generated secret in a trusted shell and use it for both processes:

```bash
export SOARM101_BROKER_TOKEN="$(python -c 'import secrets; print(secrets.token_urlsafe(32))')"
soarm101-broker --host 127.0.0.1 --port 8765
```

In another trusted terminal with the same token:

```bash
export SOARM101_BROKER_URL=http://127.0.0.1:8765
export SOARM101_BROKER_TOKEN=... # same secret
soarm101-mcp
```

The MCP host should launch `soarm101-mcp` as its **stdio subprocess** rather
than run this interactively. stdout is protocol-only. For example, an MCP
host's conceptual server configuration is:

```json
{
  "mcpServers": {
    "soarm101": {
      "command": "/absolute/path/to/venv/bin/soarm101-mcp",
      "env": {
        "SOARM101_BROKER_URL": "http://127.0.0.1:8765",
        "SOARM101_BROKER_TOKEN": "SESSION_SECRET"
      }
    }
  }
}
```

The exact configuration format varies by host. Do not check secrets into a
repository or share them with an untrusted agent environment. In particular,
do not expose the trusted host broker token to an unrestricted shell tool
under a different authority policy. MCP tool annotations and documentation
are **hints, not authorization**; the trusted broker/SDK are the enforcement
boundary. No direct MCP HTTP server or remote access is introduced in this PR.

## Trusted-host capability profiles

This optional broker feature uses a **versioned JSON profile** selected only by
the operator starting the broker. It is loaded once and pinned for the entire
broker process lifetime. Each profile restricts which existing broker routes
may be invoked, which named cameras are accessible, and optional stricter
single-command joint/model-coordinate displacement ceilings.

For example, start the trusted broker with:

```bash
soarm101-broker --host 127.0.0.1 --port 8765 \\
  --profile docs/examples/mcp-profile-coordinate.json
```

Start the stdio MCP client as usual. Before advertising tools it reads the
authenticated `GET /v1/profile` descriptor and removes disabled MCP tools.
The broker **independently enforces** the same profile on every HTTP request,
including calls not made through MCP. The profile descriptor includes the
canonical configuration's SHA-256 for experiment provenance.

See the example `mcp-profile-read-only.json`, `mcp-profile-joint.json`,
and `mcp-profile-coordinate.json` files in `docs/examples/`, plus the
`mcp-profile-supervised-manipulation.json` example for attended workspace
operation. A valid
profile names a version-1 schema, lists recognized MCP tool identifiers,
lists allowed logical cameras, and may declare `max_joint_delta_deg` and
`max_model_jog_mm`. Unknown capabilities, fields, and limit names fail
startup validation. No profile change API is exposed to agents.

A model-frame displacement ceiling is intentionally *not* a claim of
physical-distance accuracy: the SDK's calibrated physical workspace guard
and full motion safety remain authoritative. These profile ceilings can only
reduce allowed actions. Read-only profiles exclude motion routes; human-only
arming, physical STOP access, and normal GUI/CLI authority remain outside
the profile. Local broker health/profile discovery remain available for
service administration, regardless of the MCP tool subset.

The supervised-manipulation profile exposes overhead and wrist cameras,
`go_pose`, `sleep`, `jog_cartesian`, `jog_joint` (up to 10 degrees per command),
`move_gripper`, and STOP/HOLD. It intentionally has **no**
`max_model_jog_mm` limit: the existing bounded agent CLI instead enforces
the independent **physical** displacement maximum of 50 mm when the
calibrated **starting** TCP height is over 100 mm, or 10 mm when at or below
100 mm. These are *ceilings*, not required movement sizes, and do not
guarantee measured distance or object clearance. At a starting height only
slightly above 100 mm, a 50 mm downward command may cross that threshold;
plan smaller steps near obstacles or the threshold.

The profile does not control movement speed. The trusted broker can
now pin requested agent Cartesian and joint jog velocities/accelerations
at startup with `--agent-cartesian-speed-mm-s`,
`--agent-cartesian-acceleration-mm-s2`,
`--agent-joint-speed-deg-s` and
`--agent-joint-acceleration-deg-s2`. Defaults preserve historical
10 mm/s, 40 mm/s² and 8 deg/s, 25 deg/s² respectively. The broker
passes these values into the bounded agent CLI adapter, which validates
them again against the existing maximum motion envelope before hardware.
`robot_capabilities` exposes the effective `broker_requested_motion`
rates. See [agent-broker.md](agent-broker.md) for an example.
This changes configuration ownership, **not** the default physical
speed. Increasing it on hardware remains deferred pending investigation
of measured physical-height drift and rejected repeat jogs.

## MCP tool surface (initial implementation)

| MCP tool | Existing broker route |
| --- | --- |
| `robot_health` | GET /v1/health |
| `robot_capabilities` | GET /v1/capabilities |
| `robot_state` | GET /v1/state |
| `capture_camera(overhead\|wrist)` | POST /v1/capture |
| `go_pose(name)` | POST /v1/go-pose |
| `jog_joint(joint, delta_deg)` | POST /v1/joint |
| `jog_cartesian(frame, x_mm, y_mm, z_mm)` | POST /v1/jog |
| `move_gripper(open\|close)` | POST /v1/gripper |
| `sleep()` | POST /v1/sleep |
| `sleep_up()` | POST /v1/sleep-up |
| `stop()` | POST /v1/stop |

`jog_cartesian` is **translation-only**. `world` means the SDK model/base
frame, **not automatically a measured physical up/down/right/left frame**.
Read `robot_capabilities` for the calibrated `world_directions` mapping
before using human directions. `tool` means the current gripper/TCP frame.
The same physical workspace calibration and jog constraints enforced by the
broker/SDK still apply.

Camera capture returns MCP text metadata (including broker request ID and
trusted SHA-256) plus an MCP JPEG image. The adapter verifies the image hash
before returning any pixels and excludes host paths and camera device paths.
There is no independent camera session, image generation, perception, or IK.

`stop` remains available without a motion lease, but software STOP/HOLD
is **not** a hardware emergency stop. A human must explicitly arm through
`soarm101 agent arm` before any physical movement; MCP does not expose
arming, disarming, relax, calibration, servo registers, unrestricted joint
vectors, shell execution, or new motion primitives.

A Cartesian jog may internally replan when its measured joint start
shifts after path validation; the SDK keeps the strict start check
and revalidates the full path up to two additional times before
returning an error. Agents must not force or manually repeat a
persistent refusal. This is a safety-preserving pre-execution
recovery, not a guarantee of physical Cartesian accuracy.

## MCP-first live robot tool selection

At MCP initialization the adapter sends **server instructions** preferring the
connected `soarm101` tools for live robot state, permitted camera images, saved
poses, Sleep and bounded movement, without requiring the operator to say
"use MCP". Live camera requests should use `capture_camera`, not host camera
CLI/device discovery; an unavailable camera or tool must not prompt a shell
fallback around broker permissions. Tool descriptions reinforce this selection.

Server instructions are **hints**: MCP hosts can choose whether to present or
follow them. The repository's `AGENTS.md` gives the same MCP-first rule to
Codex operating within this checkout. Direct-host Codex launched elsewhere
may need an equivalent entry in `~/.codex/AGENTS.md`. This preference applies
to live robot operations, not code development or operator diagnostics;
it cannot grant authority or make the host's shell tools safe.

## MCP guidance resources and task prompt

The MCP server also publishes static, harness-neutral guidance. These resources
do not call the broker and are not a source of dynamic robot state:

- `soarm101://guidance/core` — authority boundary and evidence-first loop
- `soarm101://guidance/coordinates` — calibrated world/tool-frame use
- `soarm101://guidance/joints` — small named-joint corrections
- `soarm101://guidance/vision` — fresh-camera evidence rules
- `soarm101://guidance/recovery` — rejection/failure recovery

The `operate_robot_task(task, strategy)` MCP prompt gives hosts a compact
entry point for a task. `strategy` may be `auto`, `coordinates`,
`joints`, or `observe-only`. This is an **experimental reasoning
instruction**, not an enforcement switch: actual available actions still come
from the broker profile and Motion SDK. This distinction lets Forge Bench
compare how semantic guidance and tool surfaces affect agent performance
without conflating prompts with physical authority.

Harness-specific instructions remain in the existing OpenShell skills because
image inspection differs by harness (for example Hermes `vision_analyze`
versus Codex `view_image`). The MCP resources deliberately do not claim a
specific model or image tool.

## Canonical OpenShell agent integration

The built-in Hermes and Codex OpenShell adapters support an optional
`--interface mcp` on `soarm101 agent sandbox run`. Use
`--capability-profile FILE.json` to pin the trusted-host broker's permissions
for one run. `--read-only` reduces those permissions at the broker even if
the file permits motion. `robotctl` remains the default and a useful A/B
baseline; MCP never replaces the broker safety path.

See [agent-sandbox.md](agent-sandbox.md) for exact read-only startup,
configuration, evidence, and later supervised physical checks.

## Validation and next steps

```bash
pytest tests/test_mcp_server.py tests/test_broker.py tests/test_robotctl.py
ruff check src/soarm101_motion/mcp_server.py tests/test_mcp_server.py
```

Validate the MCP `tools/list`, `robot_health`, `robot_capabilities`,
`robot_state`, and `capture_camera` responses before any motion trial.
No hardware motion is required for the automated tests.

MCP resources and prompts are optional agent guidance; they do not supply
dynamic robot state or enforce motion policy. Specialist perception models and
Forge Bench orchestration remain separate work. An MCP host cannot bypass
the trusted broker by discovering another tool.
