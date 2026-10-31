# SO-ARM101 robot MCP through Forge Puppeteer

**Status:** SDK cleanup branch implementing the cross-repository handoff.
The SDK no longer owns or installs a robot MCP server. The native
`SOARM101` Python library, CLI, GUI, simulation and authenticated
broker remain independently usable. Optional MCP operation requires
[Forge Puppeteer](https://github.com/AgenticForge-Labs/forge-puppeteer)
on the trusted client host, **not** AgenticForge Platform or Studio.

## Responsibility

```text
Any agent using MCP          optional Hermes / Codex inside OpenShell
             |                     |
             +---------------------+
                         |
                Forge Puppeteer MCP (stdio)
                         |
                authenticated SDK broker (HTTP)
                         |
        SDK broker policy, limits, lease, fault, STOP/HOLD
                         |
                SO-ARM101 Motion SDK
```

The MCP server lives in `forge_puppeteer.mcp.launcher` and the SO-ARM
adapter in `forge_puppeteer.mcp.soarm101`. The trusted SDK broker
remains the **sole authority** over physical motion, calibration,
session ownership, watchdog, safety and trace evidence. Never give a
reasoning model a raw servo API, camera device or supervisor token.

## Independent SDK

```bash
python -m pip install soarm101-motion-sdk
soarm101 --help
soarm101-broker --help
robotctl --help
```

No Puppeteer or MCP package is required for those interfaces.

## Optional standalone MCP

Install the independently published/checked-out Puppeteer project in
the **agent's trusted host Python environment** with its MCP extra:

```bash
cd ~/code/forge-puppeteer
python -m pip install -e '.[mcp]'
```

Install the SO-ARM SDK in the same host environment, or use a compatible
installed wheel. The Puppet-owned MCP process loads the SDK's
`BROKER_CAPABILITY_ROUTES` registry but never imports raw motor
controllers for its broker transport. The host operator must separately
start an authenticated SDK broker and select a fixed scope.

Use `forge-puppeteer-mcp --config /path/to/private-config.json`.
The private configuration should select `soarm101_broker`, pin one
`broker_url`, `simulation_only`, immutable tool grants, and an
environment variable **name** for `SOARM101_BROKER_TOKEN`. Do not put
credentials in tool requests, config files, process arguments or source.
The [Puppeteer direct-broker documentation](https://github.com/AgenticForge-Labs/forge-puppeteer/blob/main/docs/SOARM101_DIRECT_BROKER_MCP.md)
contains the simulation-only example and test commands.

The MCP schema is **not wire-compatible** with the former SDK
`soarm101-mcp` script for all motion actions: Puppeteer uses
`action_id`, `caller_id`, `session_id`, and typed parameters to
preserve ownership and action/replay provenance. Update agent tool
calls; do not simulate compatibility by bypassing the trusted broker.

Review live `robot_capabilities`, `robot_state`, `list_assets` and
guidance resources where allowed. Motion is available only with a
separately authorized broker lease, validated calibration/profile,
and specific MCP startup grants; tool discovery is not authorization.
Camera results require real JPEG pixels and matching broker SHA-256.
Replay requires immutable asset revision, request/session identities,
simulation-only admission and SDK-managed STOP/HOLD fencing. Unknown
motion outcomes must not be blindly retried.

## Optional NVIDIA OpenShell agent sandbox

The SDK continues to own `soarm101 agent sandbox run` because it
creates the isolated agent, starts/supervises its host broker, pins
its capability profile, protects host STOP, and retains camera and
interaction evidence. For `--interface mcp`, it now delegates
client-only packaging to `forge_puppeteer.mcp.openshell_bundle`:

- Stage installed Puppeteer Python modules plus only the pure
  `soarm101_motion.capability_profile` source. No host SDK motion
  module, local controller, calibration tree, serial device, or
  supervisor credentials enter the sandbox.
- The Puppeteer MCP server runs in the isolated
  `/opt/soarm101-mcp` virtualenv; Docker images preinstall
  only MCP, Pydantic, AnyIO, PyYAML and JSON Schema dependencies.
- The client config selects the fixed `host.openshell.internal`
  host on the operator-selected port, explicitly enabled only
  for the OpenShell case. OpenShell controls permitted HTTP
  methods and paths; HTTP redirects and environment proxies
  are refused by the broker client.
- Trusted host-injected `SOARM101_BROKER_TOKEN` is not written
  into the config or the output evidence. Any mismatch during
  the preflight MCP tool handshake stops before an agent task.
- A read-only agent needs **no** motion authority. Physical
  experimental motion still requires all SDK-side lease,
  supervision and safety gates and is not qualified for unattended
  operation.

Simulation-only example:

```bash
soarm101 agent sandbox run --agent codex --auth installed \
  --interface mcp --sdk-simulation --task TASK.md \
  --output runs/codex-sdk-sim
```

The host running this command must have Puppeteer installed. It does
not require Platform or Studio. Docker and OpenShell are optional
deployment choices; they are **not** requirements for ordinary SDK use.

See [Agent sandbox](agent-sandbox.md), [Trusted broker](agent-broker.md),
[Puppeteer migration inventory](puppeteer-mcp-migration.md) and
[Safety](safety.md).

## Acceptance and limitations

The former SDK `soarm101-mcp` launcher, `[mcp]` extra and
SDK-owned MCP-only modules/tests are removed **on this branch**.
The migration has not yet passed actual local installed-wheel,
MCP protocol and OpenShell Hermes/Codex acceptance. Use the
Puppeteer and SDK simulated regression commands in the PR handoff.
No physical hardware or connected cameras were operated, and
the OpenShell integration is not a certified safety controller.
