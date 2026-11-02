# SO-ARM101 MCP consolidation — SDK source migration and acceptance

**Status:** The historical SDK-owned MCP code is deleted in the
`agent/remove-sdk-owned-mcp` branch; **this is not yet merged or
locally accepted**. In Puppeteer, direct-broker MCP implementation
PR #4 and isolated OpenShell client packaging/fix PRs #5–#6
are already merged. Real OpenShell Hermes/Codex, clean installed-wheel,
simulated broker and actual image-response acceptance remain
outstanding. Keep existing installed SDK environments operating
until this cleanup is tested and merged.

## Ownership

```text
Any compatible agent                      Human Python / CLI / GUI
      |                                            |
Forge Puppeteer MCP (optional)                     |
      |                                            |
      +---- SDK authenticated broker <-------------+
                     |
  SDK profile, robot state, calibration, session/lease,
  replay policy, supervisor watchdog, STOP/HOLD
                     |
               Motion SDK / robot
```

This SDK is fully usable without Puppeteer, MCP, Platform, Studio,
Docker or OpenShell. An operator who wants MCP additionally installs
`forge-puppeteer[mcp]`; an operator who wants isolated
`soarm101 agent sandbox --interface mcp` installs the same
optional Puppet package in the **trusted host** environment.
OpenShell is strictly an optional agent isolation mode, not a motion
controller or precondition for using the SDK.

## What remains in the SDK

- Python motion planning, calibrated robotics, GUI/CLI and USB camera.
- `robotctl`, trusted broker, `CapabilityProfile` and canonical
  `BROKER_CAPABILITY_ROUTES` as the only broker route registry.
- Full asset catalog, immutable recording/sequence revisions,
  simulation replay and deterministic preflight (merged PR #97).
- Watchdog, human authority, token/lease isolation, STOP/HOLD,
  fault quarantine and independent host supervisor.
- Host-side `agent_sandbox.py`: starts/supervises the SDK broker,
  provisions OpenShell, controls capability profiles, retains
  evidence and requests host-owned STOP during teardown.
- Native `soarm101`, `soarm101-gui`, `soarm101-broker`,
  `robotctl` and normal simulation modes. No new dependencies.

## Moved to Puppeteer

| Historical SDK component | Replacement |
| --- | --- |
| `mcp_server.py` | `forge_puppeteer.mcp.server`, `soarm101`, `launcher` |
| `mcp_guidance.py` | `forge_puppeteer.mcp.soarm_guidance` |
| `agent_mcp_runtime.py` | `forge_puppeteer.mcp.openshell_bundle` |
| SDK `[mcp]` extra and `soarm101-mcp` command | `forge-puppeteer[mcp]` and `forge-puppeteer-mcp` |
| SDK MCP server/guidance tests | Puppeteer MCP/broker/installed-wheel tests |
| SDK OpenShell client-only source copy | Puppet bundle stages Puppet source + SDK's pure capability registry |

The host-side sandbox remains SDK-owned because it alone knows the
current hardware/session authorization and the independent STOP
controller. It imports Puppet packaging **only when the user explicitly
selects MCP**; ordinary native SDK and `robotctl` usage need not
import Puppet.

## Runtime and safety contract

- One MCP server and one SDK trusted broker; no double MCP proxy,
  duplicate controller, or raw motor SDK import in the sandbox.
- MCP only exposes an operator-pinned subset of the live broker
  capability profile. The broker rechecks every command.
- Puppet tool inputs differ from old SDK MCP motion signatures:
  agent calls need explicit `action_id`, `caller_id`,
  `session_id`, typed parameters, and the shared action lifecycle.
  Update existing agent prompts/configs; do not pretend wire
  compatibility exists.
- MCP `capture_camera` verifies actual JPEG SHA-256; returned
  metadata excludes host paths. SDK retains independent capture
  evidence on the host.
- Replay needs `request_id`, `session_id`, immutable revision
  and SDK simulation admission; physical replay remains prohibited.
  No automatic retries after ambiguous motion outcomes.
- Software STOP acknowledgement alone is not a safety-rated
  emergency stop. Preserve host-supervisor token separation and
  robot torque/authority safeguards.
- Docker images' isolated MCP venv installs only MCP, Pydantic,
  AnyIO, PyYAML and JSON Schema dependencies. The staged
  Puppeteer source package uses the pure SDK broker registry, not
  its hardware API, private data or Python SDK installation.
- OpenShell's fixed host endpoint is `host.openshell.internal`,
  selected by trusted operator config and explicitly permitted
  only in this sandbox mode. OpenShell route/port policy applies;
  the MCP transport refuses redirects or environment HTTP proxies.

## Mandatory qualification before merging this SDK cleanup

**Source PR dependencies:** SDK main after #97/#98; Puppeteer main
after #4/#5/#6. Confirm actual SHAs again before running.

First, in a clean checkout of the SDK cleanup PR with the separately
installed sibling Puppet package:

```bash
python -m pytest tests/test_broker_mcp_independence.py \
  tests/test_capability_profile.py tests/test_agent_sandbox.py \
  tests/test_broker_replay.py tests/test_broker_watchdog.py -q
python -m pytest -m 'not hardware' -q
ruff check src/soarm101_motion tests/test_agent_sandbox.py
python -m build --wheel
```

And in Puppeteer main:

```bash
uv run --extra mcp --extra dev pytest \
  tests/robots/test_soarm101_broker_http.py \
  tests/robots/test_soarm101_openshell_bundle.py \
  tests/robots/test_soarm101_mcp.py \
  tests/robots/test_packaged_launcher_soarm.py -q
uv run --extra mcp --extra dev pytest -q
bash scripts/soarm101-wheel-smoke.sh
```

Then **locally** qualify isolated OpenShell Hermes and Codex in
read-only and SDK simulation modes with temporary tokens:

1. Build updated Hermes and Codex images from this SDK's Dockerfile
   assets; verify the installed venv has no native motor SDK.
2. Verify Puppet's client-only modules/config are the uploaded bundle
   and the old `soarm101_motion.mcp_server` is absent.
3. Check SDK broker/profile discovery, named-camera pixels and
   SHA-256, denied capabilities, denied unauthorized methods,
   session/request identity and no duplicate hardware owner.
4. Validate failed broker connection and removed/expired grant
   fail *before* agent execution, without credential leakage.
5. Validate bounded simulation action/replay, stale revision failure,
   unknown-result fence, watchdog timeout and host independent STOP.
6. Ensure an unarmed read-only agent **does not** request a motion
   authorization and cannot issue motion or STOP tools.
7. Ensure actual supervisor token, serial devices, local calibration,
   host private paths and controller modules are never mounted.
8. Check direct SDK Python/CLI/GUI/broker still work *without*
   Puppet/MCP installed. No automatic physical hardware operation.

Tests have **not** been run in this GitHub-only work session. Codex
must report exact checkout SHAs, test/skip counts, docker build
outcomes, tool discovery and rejection samples, unresolved questions
and any safe fixes made to the **same cleanup PR**.

Only after this qualification should the SDK cleanup be merged.
Do not start the dependent StackChan adapter while this cleanup
remains open. Do not activate Platform PR 6 gateway to physical robots
based on source merge alone.
