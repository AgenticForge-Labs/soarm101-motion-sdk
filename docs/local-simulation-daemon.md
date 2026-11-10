# Local persistent SDK simulation daemon

The SDK can keep one simulated arm session open behind the existing
`SDKAgentExecutor`, `RobotBrokerService`, and `BrokerWatchdog`. It is a useful
operator-run lifecycle for local client development. It opens no serial port,
does not enable torque, and cannot provide physical motion authority.

```bash
soarm101-sim-daemon start
soarm101-sim-daemon status
soarm101-sim-daemon stop
```

The default endpoint is `http://127.0.0.1:8765`. `start --port PORT` changes
the port. The daemon creates a private bearer token and PID record under
`~/.local/state/soarm101/sim-daemon` (or a supplied `--state-dir`). Keep this
directory private. A host-owned heartbeat maintains the existing two-second
watchdog deadline; the HTTP client cannot pulse that heartbeat, and expiry
fences further simulated motion. Shutdown
attempts the existing SDK STOP and closes the executor and watchdog.

Clients use the existing bounded broker contract and the private token. For
example, the existing `robotctl` can inspect the persistent session:

```bash
robotctl --base-url http://127.0.0.1:8765 --token-file \
  ~/.local/state/soarm101/sim-daemon/broker.token state
```

## Puppeteer direct-broker MCP client

Puppeteer can use this daemon as a separate SDK broker process through its
existing fixed `BROKER_CAPABILITY_ROUTES` HTTP client. This is the standalone
`soarm101_broker` MCP backend: one local daemon owns one simulated SDK session,
and Puppeteer's stdio MCP server delegates health, state, bounded `jog_joint`,
and STOP over authenticated loopback HTTP. See
`forge-puppeteer/docs/SOARM101_DIRECT_BROKER_MCP.md` and run its
`scripts/soarm101-wheel-smoke.sh` for an installed-wheel end-to-end check.

This does **not** use Puppeteer's `ConversationEnsemble`,
`SOARM101Choreography`, or any ensemble launcher. Those are a distinct
room-owned execution architecture; do not run a daemon beside an ensemble
that owns the same arm. The daemon's profile currently grants only the
bounded simulated `jog_joint` action plus reads and STOP. It does not grant
pose, Cartesian, gripper, replay, camera, or physical operations.

This service is deliberately simulation-only. Its bounded joint request is a
test input to the existing **agent-shaped broker route**, with simulation
identity and no lease; it is not a local operator choreography authority and
does not grant an agent a physical lease. Physical agent routes continue to
require the existing operator-issued, calibration-bound lease. Puppeteer's
room should continue to own its own single SDK executor/watchdog for the room
lifetime; do not start this daemon alongside a Puppeteer room that owns the
same arm. The room's trusted choreography needs a separately implemented
trusted-host SDK capability if it is to run lease-free physical actions.

Persistent physical local intent needs an explicit trusted authority and
operator-attended hardware qualification for readiness, STOP/HOLD, resource
ownership, heartbeat expiry, and teardown. This daemon implements neither
that authority nor those physical guarantees. Simulation tests are software
evidence only.
# Puppeteer v4 lifecycle integration

Puppeteer v4 may invoke the installed `soarm101-sim-daemon start` and `stop`
commands when the selected SO-ARM101 robot file explicitly sets
`mode: "sdk_simulation_daemon"`. The launcher uses a private, unused state
directory and matching loopback port, waits for this daemon's health response,
and calls `stop` only for a daemon it started. Existing or ambiguous state is
not adopted. The daemon remains the owner of the SDK session, watchdog and
heartbeat; Puppeteer acts as an authenticated broker client.

This daemon's built-in profile may expose bounded simulated SDK actions, but
the Puppeteer v4 sample grants only state, capabilities, and STOP. It does not
load a trusted choreography catalog and cannot authorize physical motion. Use
the SDK broker's separately configured simulation-actions service for exact
pinned trusted choreography; physical trusted choreography remains unsupported
and must fail closed.
