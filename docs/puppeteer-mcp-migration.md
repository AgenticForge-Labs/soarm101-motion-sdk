# SO-ARM101 SDK → Forge Puppeteer MCP ownership reconciliation

**Status:** Migration preparation only. The SDK's existing `soarm101-mcp`
entry point, OpenShell `--interface mcp` client bundle and their tests
remain functional during migration. **Do not delete them** until the exact
replacement from Forge Puppeteer passes the compatibility tests below.
This document records the post-PR-97 state on SDK main and the agreed
cross-repository ownership boundary; it is not a production cutover.

## Architectural contract

```text
User / Python / CLI / GUI ────────────────────────────────┐
                                                          v
Any MCP-capable agent -> Forge Puppeteer MCP-only -> trusted SDK broker
                                      |                   |
                           optional full Puppeteer       |
                           performance/coordination       |
                                                          v
                                     SO-ARM101 Motion SDK / native simulation
                                                          |
                                             validated hardware/controller
```

The SDK is independently usable with **no Puppeteer or MCP installation**:
Python motion APIs, GUI, CLI, saved poses/recordings/sequences, calibration,
basic camera access, simulation and trusted broker. It owns exactly one
motion/safety authority and one operator profile/grant policy. The
`BROKER_CAPABILITY_ROUTES` registry is owned by this SDK and currently
uses the tool identifiers already understood by `CapabilityProfile`.
The prior `TOOL_ROUTES` spelling is a **temporary alias to the same
registry**, never an independent table. The broker still independently
checks every requested route and pinned limit, regardless of whether an
MCP server advertises the tool.

**Forge Puppeteer** will own robot-facing MCP server construction, typed
tool adapters, resources/prompts and optional OpenShell **MCP packaging**.
It can operate standalone with the SDK/broker and without Platform or
Studios. In `mcp-only` mode, it must not require performance timelines,
production assets, a company identity or Platform gateway. Native robot
SDK transport and Puppeteer MCP transport must not each create hardware
owners. Puppeteer must talk to the existing authenticated broker (or
an equivalent SDK-owned trusted client), **not launch another SDK MCP
inside its own MCP and call it as a second safety controller**.

## Existing merged SDK functions that must survive

PR #97 has been merged into SDK `main`. These are **native SDK/broker**
features and must NOT move to Puppeteer:

- `asset_catalog.py`: library discovery, revision identities and
  provenance (discovery is not execution authority).
- `replay_policy.py`, `replay_preflight.py` and
  `replay_snapshot.py`: pinned content, calibration, start-state and
  replay policy checks.
- `simulated_replay.py`, `simulated_sequence.py`:
  supervised simulation-only replay and shared SDK execution.
- `broker.py`, `broker_sdk.py`, `broker_watchdog.py`:
  host-side session authority, separate supervisor STOP, correlation,
  watchdog, idempotency/quarantine and simulated/physical mode gates.
- `capability_profile.py`: pinned broker capabilities, per-camera
  grants, rates and displacements, including rejection of extra
  unsupported profile fields.
- `robotctl.py`: minimal standard-library operator/broker client.
  The native CLI remains independently usable for people or restricted
  agent experiments. It is **not** the MCP server.
- `agent_sandbox.py`: currently SDK-owned host-side OpenShell runner
  and trusted supervisor lifecycle. Do not move its hardware/STOP
  authority or host environment into an untrusted Puppeteer MCP client.
  Only the client-only MCP package/config generator should move.

The SDK has separate conservative physical requirements: the
experimental persistent physical joint/camera broker, independent STOP,
heartbeat, torque HOLD, calibration, speed and workspace validation,
and actual hardware qualification. A working simulation does not prove
physical braking or permit physical replay. Puppet cannot widen these
conditions.

## MCP pieces to migrate (current source → target)

| SDK location | Future owner | Notes |
|---|---|---|
| `src/soarm101_motion/mcp_server.py` | `forge-puppeteer/src/forge_puppeteer/mcp/` | Retain current tool names/schemas and signed camera image evidence; use existing common MCP server, no new server factory |
| `src/soarm101_motion/mcp_guidance.py` | Puppeteer MCP resources and prompts | Retain semantics of `soarm101://guidance/*` and `operate_robot_task`; guidance is not policy |
| `src/soarm101_motion/agent_mcp_runtime.py` | Puppeteer agent/client packaging | Preserve isolated client-only bundle and Hermes/Codex config without shipping Motion SDK tree, broker token in prompt, USB or supervisor STOP credentials |
| `soarm101-mcp` script and `[mcp]` dependency | Puppeteer `[mcp]` extra | Remove only when Puppeteer replacement starts and validates correctly |
| SDK MCP guidance and tests | Puppeteer MCP test suite | Port all camera, profile, prompt, guidance, replay and transport tests; keep SDK broker tests in SDK |
| `agent_sandbox.py` client bundling | SDK host runner may **call** reviewed Puppeteer packaging | Preserve SDK host broker setup, STOP on agent teardown, OpenShell policy and worker confinement; no duplicate supervisor |

The existing Puppeteer `mcp/soarm101.py`, `mcp/server.py`,
`devices/soarm101.py` and `mcp/launcher.py` already implement a
common action runtime and typed SO-ARM MCP tools. **Reconcile those
with the SDK MCP surface** instead of copying the old server wholesale
and creating another competing endpoint.

## Tool-level regression matrix

Do not claim parity merely because `tools/list` is nonempty:

- Read: `robot_health`, `robot_capabilities`, `robot_state`,
  `list_assets` only when the trusted broker profile grants them.
- Observation: `capture_camera` must retain validated SHA-256 over
  real pixels, metadata separation, no host path disclosure, and
  image content only when source pixels are observable.
- Motion: `go_pose`, `jog_joint`, `jog_cartesian`,
  `move_gripper`, `sleep`, `sleep_up`, `stop`.
  Rates may become slower only; do not auto-arm or expose servo registers.
- Replay: `replay_recording`, `replay_sequence`, both with
  current session ID, immutable revision and request identity.
  Physical replay stays disabled and ambiguous outcomes remain
  quarantined until a broker-confirmed STOP.
- Resource and guidance compatibility:
  `soarm101://guidance/core`, `/coordinates`, `/joints`,
  `/vision`, `/recovery`, and `operate_robot_task` prompt.
- Brokering: read-only and partial profiles must remove denied
  actions from advertised tools **and** broker must independently
  reject denied direct requests.
- Controls: two callers may not own the same arm; cancellation/STOP
  must preserve independent host safety and no implicit reconnect.
- Protocol: SDK native Python, CLI, GUI and `robotctl` still pass
  without the MCP dependency. Puppeteer MCP-only mode runs without
  Studios/Platform or a model provider.
- Packaging: Docker and OpenShell are **optional**. Running an agent
  sandbox must never mount raw serial, host camera device, Docker
  socket, SDK/controller tree, supervisor credentials or private paths.

## Dependency order: do not strand working features

1. **Already merged:** SDK PR #97. Preserve its entire asset/replay
   and broker safety work. Recheck its final state on `main`.
2. **This SDK preparation PR:** neutralize the source-of-truth
   terminology (`BROKER_CAPABILITY_ROUTES`, old `TOOL_ROUTES`
   alias), protect it with tests, document migration inventory.
   This deliberately **does not remove** any MCP executable yet.
3. **Puppeteer PR 6, first:** implement one MCP-only launch path
   over the SDK broker, transplant unique SDK MCP resources, prompts,
   camera and broker tools, and test compatibility on the merged
   SDK main. Avoid creating another standalone robot MCP project.
4. **SDK cleanup, only after step 3 is merged or accepted at an exact
   pinned SHA:** switch OpenShell client-only MCP packaging to the
   reviewed Puppeteer package, remove obsolete SDK `mcp_server.py`,
   `mcp_guidance.py`, `agent_mcp_runtime.py`, the entrypoint and
   SDK `mcp` extra. Update all tests/skills/docs, including those
   that use private sdk sandbox paths. The native SDK, broker, safety
   and `robotctl` do not move.
5. **Local Codex:** test SDK and Puppeteer wheels in *clean installed*
   environments with no editable sibling imports, fake broker, isolated
   native simulation and separate agent/supervisor tokens. Only
   separately approved operator-attended hardware validation follows.
6. **Platform PR 6:** connect the resulting Puppeteer MCP into
   ContextForge, initially with fake providers. No physical motion
   by automated tests.

Until step 4 succeeds, **the SDK still owns its historical MCP
implementation operationally**. Documentation must not advertise
Puppeteer as the new active server before the handoff is complete.

## Local validation for this SDK preparation branch

From a clean checkout at the exact SHA, with a dev environment installed:

```bash
python -m pytest tests/test_broker_mcp_independence.py -q
python -m pytest tests/test_mcp_server.py tests/test_broker_asset_catalog.py \
  tests/test_broker_replay.py tests/test_broker_watchdog.py -q
python -m pytest -m 'not hardware' -q
ruff check src/soarm101_motion/capability_profile.py \
  tests/test_broker_mcp_independence.py
git diff --check
```

These are source and simulation/fixture tests, not hardware qualification.
There are no live services, production tokens, OpenShell launches,
robot motions, firmware flash steps or extra paid licenses in this PR.
