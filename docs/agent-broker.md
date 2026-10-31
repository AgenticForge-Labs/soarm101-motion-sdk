# Agent robot broker

The `soarm101-broker` is the **trusted, authenticated host-side** transport boundary between a sandboxed reasoning agent and the SO-ARM101 Motion SDK. It does not invent new robot behaviors or grant independent safety authority. The host operator selects immutable profiles and motion rate ceilings; the SDK owns calibration, motion planning/guards, fault state, motor communication and STOP/HOLD.

## NVIDIA OpenShell, MCP and agent pipelines

```text
Hermes/Codex (NVIDIA OpenShell sandbox)
     → robotctl or stdio MCP client
     → profiled, authenticated host broker
     → physical default: bounded agent CLI → SOARM101 SDK → servo hardware
     OR explicit simulation: restricted relay → one persistent simulated SDK
```

OpenShell governs the agent's files, network and process access. It is not a robot safety controller. Profile filtering is enforced at broker admission in addition to truthful MCP/tool discovery. An agent cannot arm a robot, override host-selected rates, reach raw servos or recover forbidden commands by changing clients.

## Current execution paths

| Path | Behavior | Hardware authority |
| --- | --- | --- |
| Standard physical agent broker | Dispatches existing bounded `soarm101 agent` CLI processes, with human authorization and SDK guards | Only operator-armed, permitted actions |
| `--sdk-simulation-preview` | Persistent simulated SDK, read-only inspection | None |
| `--sdk-simulation-actions` | Single persistent simulated SDK session, profile-limited actions and independent STOP, private loopback controller | None |
| `soarm101 agent sandbox run --sdk-simulation` | Adds isolated NVIDIA OpenShell ingress for agent MCP/robotctl, with separate relay and host STOP tokens | None |
| Dedicated physical qualification scripts | Narrow trusted-host, attended experiments, not normal agent ingress | Operator-only and separately reviewed |

In the persistent simulation relay, the agent-facing token cannot call private host-only STOP. The trusted supervisor uses its **own token** and invokes STOP before a motion-capable sandbox exits. A session fault must not cause implicit reconnect or rearming. STOP completion in simulation is not physical braking evidence.

**No general persistent native SDK physical agent broker mode is released.** An additional watchdog and attended hardware safety gate are required for that transition. Software STOP/HOLD is not a safety-rated emergency stop; see [Safety](safety.md) and [Agent sandbox](agent-sandbox.md).

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

When the trusted host pins a requested joint/Cartesian rate below the SDK's
ordinary default, the subprocess adapter passes the narrower hard ceiling
to the bounded CLI. The CLI now lowers only its *internal default* rate to
fit that explicit ceiling so even read-only `capabilities` and `state`
requests can construct a valid configuration. PR #88's original 8 deg/s
pacing had exposed this as HTTP 409. This preserves the human-selected
ceiling and does not establish physical OpenShell motion validation.


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
rates requires restarting the broker. The default physical path remains the bounded CLI broker. Native SDK simulation
and experimental attended physical joint modes are explicit opt-ins; the latter
requires the watchdog and narrow policy described below.

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

## Experimental native joint broker and watchdog

`--sdk-physical-actions --host 127.0.0.1 --profile FILE --watchdog-timeout-s 3`
selects an attended, opt-in persistent SDK session using the persisted workstation
follower port, robot identity and calibration path. It preserves motor configuration
and human torque ownership on connect/disconnect. The default remains CLI-backed.
Only state, capabilities, approved camera capture, STOP and bounded joint actions
are eligible. A profile must explicitly permit state/capabilities/joint actions and
cap joint delta at no more than 2°. Requested joint ceilings cannot exceed 8°/s
and 25°/s². Every physical joint move requires healthy stationary human torque HOLD;
this mode never enables torque. Existing SDK full-path and fault guards still apply.

The private authenticated controller exposes `POST /v1/supervisor/heartbeat` with
an empty object and `GET /v1/supervisor/watchdog`. These are supervisor lifecycle
endpoints, excluded from the external relay regardless of its profile. The first
heartbeat requires an already connected session and active matching human lease.
It binds that exact lease; replacing it cannot extend this session. The monotonic
deadline is 0.5–10 seconds; the OpenShell runner pins 3 seconds and pulses every
0.5 seconds. Ordinary agent traffic never renews the deadline or authority.

A broker-owned thread checks at most every 0.1 seconds. Expiry, missing/corrupt/
replaced authority or a terminal SDK fault fences motion, invalidates queued and
late results through the existing STOP epoch, invokes canonical independent SDK
STOP, and revokes authority. This is a terminal session requiring human review and
restart. An explicit STOP also ends supervision. Its first result/error is retained;
there is no automatic second STOP latch or automatic rearm. Lease-file failure is
reported as unconfirmed revocation after attempting STOP. Events include
`watchdog_trip`, its cause and the STOP outcome. The endpoint status also retains
that outcome for supervisor teardown without another physical STOP.

Native simulation can opt into the same watchdog with `--watchdog-timeout-s`.
Its heartbeat requires a live, identity-bound simulation session; it does not
read, create or require a human physical lease. The watchdog still requires a
live broker process, Python scheduling, responsive SDK and host power. It cannot
stop a process after broker/host death or establish physical braking distance.
Physical watchdog qualification remains outstanding.

## Opt-in asset catalog

`list_assets` is disabled in the default profile. To enable this read-only route,
the trusted operator must select an explicit profile granting `list_assets` and
supply all three existing library locations using `--catalog-poses`,
`--catalog-trajectories`, and `--catalog-sequences`. The example profile is
`docs/examples/mcp-profile-asset-catalog.json`. Startup rejects incomplete
configuration. Callers cannot supply paths. `POST /v1/assets` accepts only
`offset` (0..10000), `limit` (1..100), and optional `kind` (`pose`, `recording`,
`sequence`), returning sorted pages and content revisions. `robotctl assets`
uses this route. The broker and any agent relay both enforce their pinned profiles.
Descriptors do not disclose host library paths and compatibility is unchecked.
Listing is not replay permission; recording replay requires the separate simulation-only grant below.
Existing experimental physical profiles remain unchanged and do not gain catalog
access. The catalog may reveal asset names and provenance to granted callers.

### Replay policy work in progress

`ReplayPolicy` validates an operator-owned, versioned mapping from exact asset
IDs to SHA-256 revisions, binds the policy to a robot and calibration, and bounds
speed scale, step count, duration, and joint start tolerance. It can compare a
complete measured start state against a requested one. This is a software
admission primitive whose recording and sequence routes are wired only to the
simulation-only broker below. Its bounds are not qualified physical operating
limits. Separately attended physical qualification remains required before any
physical replay feature.

`preflight_sequence` is an offline, non-executing validator for a conservative
subset of sequence steps. It checks the sequence and each pose/trajectory against
operator-pinned revisions and calibration provenance, bounds loops and known
recording/wait duration, records each recording entry target, and detects asset
changes during the preflight. Pose and entry motion duration is unknown, so the
reported duration is a lower bound, not a deadline guarantee. Cartesian pose
steps, joint overrides, and primitives are rejected pending explicit validation.
The result is not a motion plan or authority: runtime must recheck current state,
all assets, the whole path, rate/effort/workspace constraints, the broker profile
grant, deadline and STOP epoch. No broker replay endpoint is enabled by this
validator.

`preflight_recording` also validates one pinned recording, including its
calibration, scaled recording duration and entry position against supplied joint
and gripper snapshots. The replay policy separately bounds joint and gripper
entry error. These values are not a live-state guarantee: an eventual execution
path must reacquire measured state under broker authority and STOP fencing and
validate the entry path and every replay frame before movement. Recorded duration
excludes entry motion and settling.

Recording preflight now loads a bounded in-memory snapshot of the operator-pinned
NPZ and metadata bytes, verifies their combined revision, checks archive and
metadata identity, and constructs validated trajectory samples from those bytes.
A later file change cannot alter that snapshot; a fresh load detects a changed
revision. This does not replace the required live authority and state checks or
make the snapshot itself an execution grant.

A simulation-only development helper, `replay_simulated_recording`, exercises a
pinned recording through an already-connected simulated SDK session. It requires
simulation action mode, the broker's explicit replay tool grant, an exact
operator-pinned replay policy and a live trusted watchdog heartbeat. Simulation
replay is unarmed and does not create or require a human physical lease. It checks
matching simulation identity, pinned content and entry state, and validates
planned joint samples against broker rate ceilings. It uses no entry move and
rejects a start outside the controller's own tolerance. A deadline requests the
existing STOP path and late completion is rejected. The broker route below reuses
this helper. Physical replay remains unavailable.

After simulated replay has attempted enabling, a planning or playback failure
requests the same independent STOP path and reports its separate outcome. An
already-pending or failed STOP remains unconfirmed rather than being reported as
cancelled or safe. These semantics are software-only and do not qualify hardware.


### Supervised recording replay (simulation only)

`POST /v1/replay-recording`, `robotctl replay-recording` and the MCP tool
`replay_recording` require an explicit profile grant. Default MCP/profile tools
exclude replay. The broker requires `list_assets`, `stop`, all three configured
library paths, `--sdk-simulation-actions`, `--watchdog-timeout-s` and
`--replay-policy /operator/policy.json`. The immutable `ReplayPolicy` schema v1
requires `robot_id`, `calibration_id` (`simulation`), `allowed_assets` (an object
mapping exact catalog IDs to `sha256:…` revisions), `max_speed_scale` (0..1),
`max_steps` (1..1000), `max_duration_s` (0..300), `max_start_error_rad` (0..0.1)
and `max_start_gripper_error` (0..0.1), with positive bounds. No caller supplies
paths, policy, entry moves, rate overrides or sequence steps. Restricted physical
modes reject replay configuration; no physical replay is enabled.

The complete payload is `request_id`, `session_id`, `asset_id`, `revision` and
`speed_scale`. Obtain `session_id` from current state `broker_session.id` or
capabilities `recording_replay.session_id`. The request ID is a caller-chosen
1..128 character ASCII identifier containing letters, digits, `.`, `_` or `-`.
The SDK simulation session must already exist with a live trusted supervisor
heartbeat. The exact profile grant and replay policy authorize this simulation
route; no human lease is required or created. Playback reuses that session, the
ordinary broker lock and independent STOP,
loads a fresh hash-verified in-memory snapshot, checks calibration and live
joint/gripper entry state, and runs SDK planning, workspace, rate, effort, fault
and completion checks. It never moves to the entry pose automatically. Joint
samples obey broker speed/acceleration ceilings; gripper writes receive the
broker's pinned raw speed and acceleration. The playback deadline starts after simulated enable and includes SDK
planning, replanning, playback and settling. It is a software
deadline dependent on Python scheduling and responsive SDK STOP, not a real-time
guarantee.

Repeated IDs with the same payload return the retained terminal response, even
after STOP within the current broker process. While pending they return conflict
with unknown outcome; changed payloads conflict. Responses and pending
reservations are retained for the SDK session lifetime, up to 1000 IDs with no
eviction. Capacity exhaustion rejects new IDs. Durable reservation records also
bind caller IDs to their original payload across broker restarts. A restarted
broker rejects reused IDs because it cannot reconstruct the original response;
it keeps all replay quarantined until a later confirmed simulated STOP is
durably recorded. A damaged or truncated journal also quarantines replay until
that reconciliation record. Never assign a new ID automatically to retry an
unknown action. Broker audit events retain caller/session/asset/revision identity. A failed action after enable attempts
STOP and returns separate `stop.outcome` evidence (`confirmed` or `unconfirmed`).
STOP, lease expiry/replacement/revocation and watchdog failure fence future work;
queued work and late success are rejected without rearming or resuming.

Pose-only replay, automatic entry motion, durable terminal response lookup,
OpenShell runner forwarding of replay policy/library configuration, packaged
stdio/clean-install replay acceptance and physical qualification remain gaps.
Sequence execution uses an ordered preflight result, re-resolves pinned
dependencies before enable and before each step, and rechecks measured start
state, rate, watchdog, lease and STOP fencing at runtime. Saved poses use the
SDK's guarded joint path; recordings use the existing guarded replay without
automatic entry movement; gripper and wait steps retain their declared order.
This route remains simulation-only and is not hardware qualification.
