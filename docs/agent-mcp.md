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
and `mcp-profile-coordinate.json` files in `docs/examples/`. A valid
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

## Validation and next steps

```bash
pytest tests/test_mcp_server.py tests/test_broker.py tests/test_robotctl.py
ruff check src/soarm101_motion/mcp_server.py tests/test_mcp_server.py
```

Validate the MCP `tools/list`, `robot_health`, `robot_capabilities`,
`robot_state`, and `capture_camera` responses before any motion trial.
No hardware motion is required for the automated tests.

This PR intentionally does **not** implement per-agent capability profiles,
MCP skills/resources/prompts, specialist perception models, OpenShell packaging,
or Forge Bench orchestration. Those are separate, dependent PRs. Do not assume
an MCP host can bypass the broker by discovering a different tool.
