# Agent robot broker

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
