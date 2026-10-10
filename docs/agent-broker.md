# Agent robot broker

## Operator-only direct physical session test (PR #88)

A separate local example, `examples/persistent_broker_motion_trace.py`,
can create a **narrow loopback-only physical trial broker** whose
only moving routes are `/v1/go-pose`, `/v1/sleep`, and
`/v1/stop`. Every request still requires the broker bearer
token; motion also requires a valid human-issued calibration-bound
lease. The client tests the full five-pose sequence, and the
broker retains a stable in-process SDK connection throughout.
The operator can set joint rates and choose `joint_only=True`
for this trial to avoid commanded gripper closure; there are
no speed-policy carveouts. Any larger agent surface or
physical persistent production startup remains **disabled**.

Normal broker startup remains `AgentCommandExecutor` (CLI
subprocesses) and `--sdk-simulation-preview` continues to
deny every POST. This experiment alone does not establish
the required hardware STOP, ownership, and Cartesian tests.


## PR 2 experimental read-only SDK session

The normal broker still invokes the bounded agent CLI; do not deploy the
SDK preview as a physical-motion service.

For safe, local simulated inspection only (requires a broker token):
```bash
SOARM101_BROKER_TOKEN="$(python3 -c 'import secrets; print(secrets.token_hex(24))')" \
  soarm101-broker --sdk-simulation-preview --host 127.0.0.1 --port 8765
```

This mode creates one simulated SDK session lazily and supports only the
authenticated health, profile, state and capabilities GET endpoints. It
rejects every POST, including STOP and all movement/camera actions; no serial
port is opened and no torque is enabled. It exists to validate the persistent-session/transport approach.
PR #88's internal simulated HTTP tests exercise authorization, STOP
while a fake joint move is in progress, queued-command cancellation and
lease revocation; this cannot establish safe Feetech
bus cancellation, cross-process ownership, physical workspace correctness,
or hardware HOLD success. Those require separate validation on this PR
before activating a physical SDK executor.
The standard broker behavior remains unchanged until a supervised
physical validation and merge.



MCP-capable local agents can optionally use the stdio adapter described in
[agent-mcp.md](agent-mcp.md). MCP is another client of these routes, not a
second motion or authorization implementation.

The broker is a narrow host-side transport boundary for sandboxed reasoning agents.

It does not define new robot behavior. It serializes HTTP/JSON requests and delegates every
physical action to the existing bounded `soarm101 agent ...` CLI on the trusted host. The
Motion SDK therefore remains authoritative for calibration identity, human-issued authority
leases, motion validation, workspace policy, hold/stop behavior, and named camera capture.

## Trust boundary

```text
sandboxed agent
  -> robotctl
  -> HTTP/JSON
  -> soarm101-broker on the trusted host
  -> soarm101 agent ...
  -> Motion SDK
  -> hardware
```

The sandbox should not receive the Motion SDK source tree, the unrestricted `soarm101`
executable, serial devices, camera devices, Docker sockets, SSH keys, or unrelated host files.
The sandbox-side `robotctl` client is standard-library-only and may be copied into an
isolated worker without installing this package.

Human arming remains outside the broker:

```bash
soarm101 agent arm --minutes 30
```

The broker intentionally exposes no remote arm, disarm, relax, calibration, configuration,
raw-joint-vector, servo-register, or arbitrary-shell endpoint.

## Trusted motion envelope

The broker inherits no motion policy from the sandbox. At startup, the trusted host selects
the Motion SDK envelope. Defaults use the shared 100/1000 convention:

```text
joint:        100 deg/s, 1000 deg/s^2
TCP linear:   100 mm/s, 1000 mm/s^2
TCP angular:  100 deg/s, 1000 deg/s^2
```

The broker injects those exact limits into every bounded `soarm101 agent` motion
subprocess. `GET /v1/capabilities` reports the effective envelope. `robotctl` intentionally
has no request fields or CLI options that can widen it.

An operator can choose a lower trusted-host envelope when starting the broker, for example:

```bash
soarm101-broker \
  --max-joint-speed-deg-s 80 \
  --max-joint-acceleration-deg-s2 500
```

The remaining linear/tool-angular values stay at their broker defaults unless explicitly
set by the trusted host.

## Operator-owned motion rates (all broker movement)

The trusted human chooses one **maximum** speed and acceleration for each
physical unit family at broker startup, separate from the SDK's absolute
100/1000 hardware envelope. These rates are not per-agent privileges. Every
broker movement uses them by default, with optional *slower* values:

- Joint-space motion, including **saved poses, Sleep and sleep_up, and jog_joint**:
  `--agent-joint-speed-deg-s` (default 8°/s) and
  `--agent-joint-acceleration-deg-s2` (default 25°/s²).
- TCP translation jog: `--agent-cartesian-speed-mm-s` (default 10 mm/s) and
  `--agent-cartesian-acceleration-mm-s2` (default 40 mm/s²).
- Stock gripper actions, including the gripper portion of saved poses and
  Sleep: `--agent-gripper-speed-raw` (default 250) and
  `--agent-gripper-acceleration-raw` (default 20).
  These Feetech servo parameters are **not** joint degrees/s or mm/s.
  Raw speed 0 means maximum and is deliberately rejected.

```bash
soarm101-broker --host 127.0.0.1 --port 8765 \
  --profile docs/examples/mcp-profile-supervised-manipulation.json \
  --agent-joint-speed-deg-s 15 \
  --agent-joint-acceleration-deg-s2 150 \
  --agent-cartesian-speed-mm-s 10 \
  --agent-cartesian-acceleration-mm-s2 40 \
  --agent-gripper-speed-raw 250 \
  --agent-gripper-acceleration-raw 20
```

The 15°/s / 150°/s² numbers are an **operator-selected candidate**,
not an automatically validated physical profile. The observed successful
joint-only supervised route ran at 8°/s and 25°/s². A broker's
`GET /v1/capabilities` exposes both `broker_requested_motion`
(for older clients) and `broker_motion_rate_limits`. The same rate
ceilings are supplied as the bounded CLI's joint/linear motion limits,
so a saved pose cannot silently revert to hard-coded 8/25 or exceed the
human-selected limits.

An agent may omit rates and receive the broker defaults, or include
lower positive rates. For instance, `go_pose`, `sleep`,
`sleep_up` and `jog_joint` accept optional
`speed_deg_s` and `acceleration_deg_s2`; `jog_cartesian` accepts
`speed_mm_s` and `acceleration_mm_s2`. Saved poses, Sleep and
`move_gripper` accept optional `gripper_speed_raw` and
`gripper_acceleration_raw`. The broker validates each provided field
before execution. A request above the human limit, nonfinite value,
zero, or malformed integer is **rejected**, not clamped or ignored.

MCP and robotctl use the same HTTP arguments. Capability profiles still
determine *which* actions are available. The human remains responsible for
arming the robot; agents cannot change the pinned policy. Changing global
rates requires restarting the broker. The direct-SDK executor remains a
**simulation-only read-only public preview**; physical motion still goes
through the existing bounded CLI broker path.

## Start the broker

Generate a per-run token and start the host service:

```bash
export SOARM101_BROKER_TOKEN="$(python -c 'import secrets; print(secrets.token_urlsafe(32))')"
soarm101-broker --host 127.0.0.1 --port 8765
```

The default event log is:

```text
~/.local/state/soarm101/broker/events.jsonl
```

Each request receives a request ID and records the bounded action, request payload, result or
error, duration, and timestamp. Camera acquisition, host-file read, hashing, and evidence
recording are one serialized broker operation, so concurrent requests cannot race a capture
between acquisition and evidence hashing. Camera image bytes are not written into the JSONL
log. Successful captures emit a trusted `capture_evidence` record with the image SHA-256;
the host capture path and camera device path remain in the trusted host log and are not
returned to the sandbox.

## Local transport validation

Before enabling any sandbox or motion, validate the transport on the target workstation with
the follower powered but relaxed. Use two terminals and do **not** issue `agent arm` for this
gate.

Host terminal:

```bash
export SOARM101_BROKER_TOKEN="$(python -c 'import secrets; print(secrets.token_urlsafe(32))')"
soarm101-broker --host 127.0.0.1 --port 8765
```

Client terminal, using the same token:

```bash
export SOARM101_BROKER_URL=http://127.0.0.1:8765
export SOARM101_BROKER_TOKEN=...
robotctl health
robotctl capabilities
robotctl state
robotctl capture overhead
robotctl capture wrist
```

Acceptance requires authenticated health/capability/state responses, fresh named-camera
captures whose client-side SHA verification succeeds, and corresponding JSONL broker evidence.
No motion authority is required or expected. Stop the broker after the check.

## Operator-pinned capability profiles

An operator may start the broker with `--profile FILE.json` to select a
versioned restricted tool/camera/motion profile for one experimental run.
The broker enforces it on every request, not just in MCP tool discovery;
the profile and SHA-256 can be inspected via authenticated
`GET /v1/profile`. Default startup (no profile) retains the existing full
bounded broker surface. See [agent-mcp.md](agent-mcp.md) and
`docs/examples/mcp-profile-*.json`. Profiles cannot authorize unbounded
hardware control, relax calibration, or create physical authority.

## Broker routes

Read-only:

```text
GET /v1/health
GET /v1/capabilities
GET /v1/state
```

Bounded actions:

```text
POST /v1/capture
POST /v1/go-pose
POST /v1/joint
POST /v1/jog
POST /v1/gripper
POST /v1/sleep
POST /v1/sleep-up
POST /v1/stop
```

All routes require:

```text
Authorization: Bearer <SOARM101_BROKER_TOKEN>
```

Requests are serialized before delegation so two agent calls cannot open the follower serial
port concurrently.

A capture response includes capture metadata plus SHA-256 and base64-encoded JPEG bytes, but
not the host filesystem capture path or camera device path. The sandbox client verifies the returned bytes against the
trusted SHA-256 before writing the image. Default filenames include the broker request ID, so
multiple captures within the same second cannot overwrite one another. The client never needs
direct access to the host camera path.

## Sandbox client

The installed development environment provides `robotctl`, but sandbox images should copy
only `src/soarm101_motion/robotctl.py` (or an equivalent packaged standalone executable)
instead of installing the full Motion SDK.

Configure the client:

```bash
export SOARM101_BROKER_URL=http://host.openshell.internal:8765
export SOARM101_BROKER_TOKEN=...
```

Examples:

```bash
robotctl capabilities
robotctl state
robotctl capture overhead
robotctl go-pose agent_start_overhead
robotctl joint shoulder_pan --delta-deg 5
robotctl jog --frame world --x-mm 0 --y-mm 5 --z-mm 0
robotctl jog --frame tool --x-mm 0 --y-mm 0 --z-mm 2
robotctl gripper close
robotctl sleep
robotctl sleep-up
robotctl stop
```

## OpenShell / benchmark policy

The intended benchmark deployment allows only the minimum client runtime needed to reach
the broker destination. Model/provider traffic is separately mediated by the benchmark
harness. The robot serial and camera devices remain exclusively on the trusted host.

The current `robotctl.py` is a Python script. OpenShell identifies the *real executable*
that opens a connection, so a policy for this client must name the Python interpreter rather
than the script path. Such a rule therefore does not prove that a request came specifically
from `robotctl.py`; other code executed by that permitted interpreter could make the same
bounded broker requests. This is acceptable for the current design because the broker itself
is the robot-action boundary and exposes no unrestricted robot, shell, calibration, or raw
servo surface. If per-client executable identity becomes a benchmark requirement, package a
native `robotctl` executable and pin its executable hash/path in the OpenShell policy.

The bearer token is defense in depth and run identity, not the sole sandbox boundary. A
sandboxed agent may be able to read its own environment, so the execution environment must
still enforce which runtimes can connect to the broker and which host resources exist inside
the sandbox.

The broker is intentionally implemented as a thin transport over the current bounded CLI.
A future persistent in-process robot session may replace subprocess delegation if connection
overhead becomes material, but that optimization must preserve the same public broker
contract and the same Motion SDK safety ownership.
