# Testing

## SDK-owned MCP removal: Puppet-only integration validation (post #97–#98)

The SDK capability registry now has an MCP-independent canonical name,
`BROKER_CAPABILITY_ROUTES` (`TOOL_ROUTES` is the exact same dictionary
for compatibility). Preserve source safety and agent sandbox behavior
during the subsequent Puppeteer migration. Run this **software-only**
regression first, then the ordinary broker/MCP tests:

```bash
python -m pytest tests/test_broker_mcp_independence.py -q
python -m pytest tests/test_mcp_server.py tests/test_broker_asset_catalog.py \
  tests/test_broker_replay.py tests/test_broker_watchdog.py -q
python -m pytest -m 'not hardware' -q
ruff check src/soarm101_motion/capability_profile.py \
  tests/test_broker_mcp_independence.py
```

Do not interpret successful local tests as an attended physical STOP test.
The old `soarm101-mcp` remains installed and usable until a separate
Puppeteer MCP-only implementation is available and accepted. See
[docs/puppeteer-mcp-migration.md](docs/puppeteer-mcp-migration.md) for
the tool/prompt/camera/OpenShell parity matrix and dependency order.


## Current validation boundary (2026-10-21)

The software integration for **PRs #90–#93** is merged on `main`. Tests below describe the simulated native SDK broker, separate-token NVIDIA OpenShell relay, MCP profile enforcement and host-supervisor STOP, as well as dated operator-only physical investigations. **Software acceptance is not physical braking certification or approval for general persistent-SDK physical agent control.**

Standard physical agent commands remain bounded-CLI-backed. Any future native physical multi-action cutover needs its own broker-owned watchdog and local attended servo/encoder STOP qualification; **PR #94 was README maintenance, not that future gate**. These dated test sections may refer to PRs as open or draft because they preserve the original trial-time record. Do not interpret those labels as current release status.

## Historical test plans and qualification evidence

## PR #93 — SDK simulation relay + supervisor acceptance (merged; simulation only)

Without physical hardware, test restricted agent HTTP relay versus private
host STOP, two-token noninterchangeability, MCP/profile capability
projection, multiple joint actions on one simulated SDK connection,
STOP lease revocation, after-STOP denial, read-only policy and
Ctrl+C/SIGTERM STOP-before-teardown. The default CLI-backed broker
continues to behave unchanged. Real OpenShell/Hermes/Codex provider and
credential checks require local Codex; CI uses synthetic OpenShell.

```bash
ruff check .
python -m pytest --no-cov -q tests/test_broker_sdk.py tests/test_agent_sandbox.py
python -m pytest --no-cov -q
```

No real servo devices, services or deployments are part of this test.
Before any later physical cutover, the separate unassigned hardware milestone needs its own independent safety gate.


## PR #92 — persistent native-SDK simulated broker (merged)

The **only new CLI execution mode** is `--sdk-simulation-actions`. It creates
one persistent simulated SO-ARM101 SDK session and dispatches a
human-profiled subset of the existing broker HTTP API **without spawning
a CLI process for each action**. It requires a trusted, camera-free
`--profile`, includes `robot_state`, `robot_capabilities` and `stop`,
and binds **only to 127.0.0.1**. A bounded example profile is
`docs/examples/sdk-broker-simulation-actions.json`. Production and
physical-agent brokers remain CLI-backed. `--sdk-simulation-preview`
remains read-only and mutually exclusive with the new option.

No physical authority lease is used or required for simulation actions.
Multiple bounded simulated joint actions reuse one SDK session; responses
identify the session as simulation and report `armed: false`. STOP uses
the existing independent SDK dispatch, invalidates queued actions, latches
the simulation session against further motion until restart, and writes
broker events. STOP completion in simulation is **not proof of physical servo
braking**. This PR does
not change the OpenShell listener, start/stop live services, turn on
real-hardware POST, or use the trial's restricted physical capability
bypass. The ordinary physical broker does not have a `--sdk-physical`
mode. Real-world integration is a separate future physical cutover (not implemented by #93).

**Local software check (no physical hardware):**
```bash
ruff check .
python -m pytest --no-cov -q tests/test_broker_sdk.py tests/test_broker.py \
  tests/test_broker_stop_faults.py tests/test_broker_session_lifecycle.py
python -m pytest --no-cov -q
```

**Isolated simulated HTTP broker example (terminal 1):**
```bash
SOARM101_BROKER_TOKEN="$(python -c 'import secrets; print(secrets.token_hex(24))')"
export SOARM101_BROKER_TOKEN
soarm101-broker --sdk-simulation-actions --host 127.0.0.1 \
  --port 8765 --profile docs/examples/sdk-broker-simulation-actions.json
```
The `soarm101-broker` entrypoint is defined in `pyproject.toml`.
The test suite exercises full protocol behavior with an isolated
simulation lease; no real-arm arming is used.


## PR #91 — supervisor STOP lifecycle acceptance (merged; historical test plan)

CI-only tests use fake OpenShell and broker processes. They exercise
ordinary success, injected KeyboardInterrupt, STOP failure, read-only
no-motor cleanup and lease revocation before a CLI STOP subprocess. Physical CLI STOP
regressions assert no redundant torque-enable latch in a healthy HOLD
and no implicit rearm when torque is genuinely disabled.
STOP must be requested **while broker is alive and before sandbox
deletion**, even on agent error, not added to a model prompt. On STOP
failure, `supervisor-stop.json` must say `completed=false` and the
otherwise successful run must fail. CLI `completed=true` means a
bounded command returned, **not** that native physical encoder
stability was verified.

```bash
python -m pytest --no-cov -q tests/test_agent_sandbox.py tests/test_broker.py
ruff check .
python -m pytest --no-cov -q
```

**Hardware handoff (not yet authorized by this PR):** With an unloaded,
human-supervised arm, validate that Ctrl+C terminates an agent's task,
invokes trusted STOP, prevents later commands, revokes authority,
and leaves stable physical encoder HOLD. This test requires the next
explicit persistent-SDK physical agent broker integration. A legacy
CLI broker can serialize STOP behind an in-flight CLI subprocess;
do not claim a physical in-flight STOP pass from CI alone.


## Draft #90 — faster one-shot Overhead STOP qualification (2026-10-12)

**2026-10-12 preflight correction:** The first attended `overhead-single`
attempt stopped before any movement at measured shoulder-lift **−101.89°**.
The SDK's canonical Sleep is approximately **−102.05°** for that joint
and deliberately uses a **2° inset inside already safety-margined
executable joint limits**. The trial's extra 5° start/target inset
mistakenly rejected a healthy Sleep-adjacent pose. This extra inset is
removed only from the experimental Overhead preflight: both measured
positions and saved targets must still lie within the SDK's authoritative
effective calibrated limits, which already incorporate its configured
mechanical-stop margin. Calibration provenance, max 100° joint travel,
at least two joints with ≥12° travel, stationary healthy torque HOLD,
rate caps, independent STOP and post-STOP verification remain enforced.
On refusal, the summary now includes the measured joints, saved target
and executable bounds; the original failure trace is unchanged.
No physical Overhead STOP result has yet been recorded.

**Unmerged, operator-only physical experiment, not production motor control.**
New `--trial overhead-single` uses the CURRENT measured, calibrated and
stationary torque-held posture and moves **once** toward the existing saved
`agent_start_overhead` pose, joint-only (no gripper, preparatory Sleep,
Overhead roundtrip, retry, automatic recovery or REARM). The operator must
review every joint's printed displacement and swept-route clearance and
type `OVERHEAD_START`. ENTER during physical motion requests independent
broker STOP; `STATIONARY` is only for confirmed stable physical HOLD.

The speed is **6°/s, acceleration 30°/s²** by default (3x faster than
the earlier 2°/s short-pan trial); hard caps are **8°/s, 40°/s²**
and 20 Hz. The measured pose and saved Overhead must match calibrated
joint identities, remain inside SDK effective calibrated joint limits (which already incorporate its configured stop margin), require at least two
joints moving ≥12°, and no joint may travel more than 100°. The broker
accepts only one exact joint-only Overhead POST and STOP; it blocks any
second pose, other pose, Sleep, jog, gripper, rate override or post-STOP
replay. The physical joint-only pose reuses existing human torque HOLD
rather than issuing another 1/1 servo latch.

The STOP qualification requires fresh measured encoder progress from
at least **two** distinct joints with ≥2° remaining, one corrected
0/254 HOLD, ≤1° 1.5-s settling, fault-free connected torque state,
cancellation of the interrupted action, human STATIONARY observation
and revoked authority. The Feetech Moving bit can be false even as an
encoder changes: preserve it as diagnostic, not sole motion authority.
Neither software success nor a hobby arm implies certified braking.
An earlier real elbow STOP continued ~18.4°; the later short 2°/s
pan test stopped with ~0.088° settling. The multi-joint physical gate
still requires separate supervised evidence, clear workspace, no load
and a reachable physical power cutoff.

### Software validation (no hardware)

```bash
cd ~/AgenticForge/soarm101-motion-sdk
git switch feature/broker-independent-stop-faults
git pull --ff-only
git rev-parse HEAD
source .venv/bin/activate
ruff check .
python -m pytest --no-cov -q tests/test_broker_stop_trial.py tests/test_broker_sdk.py tests/test_broker_stop_faults.py tests/test_stop_ownership.py
python -m pytest --no-cov -q
```

### Attended test (only after exact-head CI, human readiness and clearance)

```bash
soarm101 agent arm --minutes 30
python examples/persistent_broker_stop_trial.py \
  --trial overhead-single --port /dev/ttyACM1 --robot-id so101 \
  --output "$HOME/soarm-motion-tests/overhead-stop-$(date +%Y%m%d-%H%M%S).jsonl"
```

Decline at `OVERHEAD_START` for a read-only pose/torque preflight;
the script reports a nonpassing qualification, with no scripted motion
when declined. Human arming itself can enable motors. After confirmation,
press ENTER **early, when more than one joint is physically moving**.
If anything behaves unexpectedly, use the accessible power cutoff
rather than retrying software STOP. Do not issue a recovery move.

Share `*.summary.json`, `*.broker-events.jsonl`, and motor `*.jsonl`
together (redact tokens/leases). Distinguish raw stop-latch timing,
braking response, and the separate 1.5-s stabilization-monitor HTTP
duration. Never edit these original evidence files.


## 2026-10-12 attended moving-pan STOP: physical response good, evidence-gate defect

The operator's `short-stop-20261012-082341` trace was collected on real
hardware. The pan encoder advanced 2091 -> 2098 ticks in 0.299 s immediately
before STOP (0.6154°, consistent with the commanded 2°/s), with 3.69° still
to go. STOP made ONE 0/254 six-motor position latch; no additional position
targets were issued afterward. Maximum post-latch drift was 0.0879°,
remaining stationary to encoder resolution through the follow-up read.
The human reported an immediate physical stop. Torque reattachment was
healthy: connected=true, torque_enabled=true, moving=false, faulted=false.

The old script nevertheless reported `passed=false`. Its last-change
timestamp mistakenly used the earlier *different* sample's time
(0.1232 s old) instead of the changed sample's time (0.0231 s old).
It also vetoed genuine moving encoder evidence merely because the Feetech
`Moving` status bit was false during slow movement. The evidence evaluator
now timestamps actual encoder transitions and records contradictory but
fresh `Moving` flags as warnings, retaining strict measured-change,
freshness, progress, remaining-travel, after-STOP settling, fault, human
authority and one-write rules. Do not edit this original trial summary;
its software `passed=false` was the *old evaluator's* result.

This is encouraging **attended 2°/s single-joint STOP evidence**, not a
qualification of the earlier higher-energy 18.4° elbow excursion.
Do not default to the multi-joint Overhead trial or release native
production motion based on it. A broader Overhead stop requires separate
operator-reviewed joint-specific bounds, workspace/clearance planning,
and physical cutoff. The present change is software-only; hardware has
not been moved by this change. See
`docs/physical-stop-short-trial-20261012.md`.


## Draft #90: preflight-only physical session teardown remains command-free

Live Feetech torque reattachment now adopts the six actual hardware
`Torque_Enable` states on connection. Because a human CLI might already
be holding the arm, a trial which only reads state and then **rejects**
Sleep preflight must not send a new servo position HOLD on ordinary
session cleanup. `SDKAgentExecutor` now records whether any actual
movement action was attempted; when none was, physical `close()`
disconnects transport without the implicit `motion.stop` position write.
This also applies when the trial simply opens and reports status before
operator declines preparation. After attempted movement, existing fault,
STOP ownership and hold behavior remain.

Software regression: `test_read_only_physical_trial_close_does_not_issue_motor_hold`
in `tests/test_stop_ownership.py`. Keep both the real hardware
`get_hardware_state` checks and the script's read-only failure diagnostics;
never bypass a continuing `Moving` or `Status` motor fault.


## PR #90 local failure: live torque state not adopted on second process

Operator trial `short-stop-20261011-211824` stopped **before any scripted
Sleep or STOP movement**, reporting `Sleep preparation requires a
stationary, healthy torque HOLD`. The displayed encoder positions were
valid and close to calculated Sleep other than shoulder-pan +7.03°.
The exact raw state flag in that trial is not yet available here because
its local trace/summary was not uploaded.

**Confirmed source bug:** `soarm101 agent arm` enabled the real Feetech
servos from one process. After that process disconnected without relaxing
torque, the trial connected via a fresh `FeetechBackend`. Its
`_torque_enabled=False` constructor flag was not reconciled to
`Torque_Enable` registers during `connect()`. The preflight checked that
stale flag and could reject a legitimate hardware hold. This defect
also prevented independently connected sessions from correctly deciding
whether the physical STOP primitive could issue a HOLD before enabling
torque through a new command.

**Code fix:** On physical connection, read every motor's actual torque
enable register. Allow all enabled or all disabled, reject partial/mixed
states, no writes. For already-enabled joint-only physical Sleep, reuse
the existing hold rather than issuing a new low-acceleration 1/1 position
latch. A genuinely moving, faulted or torque-off arm still fails closed;
the trial now saves detailed read-only per-motor failure evidence.

**Software-only next:**
```bash
cd ~/AgenticForge/soarm101-motion-sdk
source .venv/bin/activate
ruff check .
python -m pytest --no-cov -q \
  tests/test_feetech_backend.py \
  tests/test_broker_sdk.py \
  tests/test_broker_stop_trial.py \
  tests/test_stop_ownership.py
python -m pytest --no-cov -q
```

Do not retry live motion until tests, physical clearance, and state
checks are reviewed. If `Moving=true` or a fault persists after this
repair, preserve the read-only diagnostic output and do not override it.
The old 18.4° powered elbow excursion is NOT cleared.


## PR #90 revised attended STOP test — Sleep preparation then moving-servo check

**Current draft behavior (supersedes older preflight proposals below):**

The operator-reported first trial failed before movement because the broker
did not admit the short profile. The next trial failed its expected-Sleep
check, because `agent arm` holds the **current** posture, not Sleep.
The first subsequently passing short test from Sleep cancelled a host
request cleanly and measured stable encoder HOLD, but occurred near the
8° endpoint, with unchanged pan encoder and Moving=false before STOP.
Therefore **physical braking was not demonstrated**; its unchanged
original trace is described in
`docs/physical-stop-short-trial-20261011.md`.

The new default `--trial short` makes Sleep a real prerequisite action
rather than expecting it magically after `agent arm`:

1. After a calibration-bound human lease, read all joint encoders, check
   executable calibration and healthy stationary torque HOLD. Inspect the
   entire physical route to the selected Sleep and type `PREPARE_SLEEP`.
   Clear/secure the unloaded arm, cabling, table and obstacles.
2. Through the **same authenticated local physical broker** execute exactly
   one joint-only canonical `POST /v1/sleep` (no gripper close), with
   bounded rates. Verify its completion, session continuity and measured
   Sleep position. No pan action if preparation fails.
3. Then operator types `START` for a single fixed +8° shoulder-pan action.
   Press ENTER *early while movement is visibly in progress*. The broker
   still sends independent STOP, verifies 1.5-s physical settling and
   rejects completion of the interrupted action. Type `STATIONARY` only
   if the arm actually holds.
4. In addition to stable post-STOP feedback, check a passive **pre-STOP**
   tracer snapshot: latest encoder feedback and moving flag are fresh;
   pan encoder changed just before STOP; travel is >=0.5° and >=2°
   remains. A host active flag alone or a near-endpoint HOLD is
   **inconclusive**, never a physical braking PASS. No additional serial
   reads during the moving leg or autonomous retry/REARM.
5. A new operator trial is not authorized if clearance is uncertain.
   Invalid calibration, motion/effort/transport errors, faulted status,
   STOP timeout, or repeated STOP goals are blockers.

Software-only first:
```bash
ruff check .
python -m pytest --no-cov -q tests/test_broker_stop_trial.py \
  tests/test_broker_sdk.py tests/test_stop_ownership.py \
  tests/test_broker_stop_faults.py
python -m pytest --no-cov -q
```

The attended physical test remains a *separately approved* local step.
```bash
soarm101 agent arm --minutes 30
python examples/persistent_broker_stop_trial.py \
  --trial short --port /dev/ttyACM1 --robot-id so101 \
  --output "$HOME/soarm-motion-tests/short-stop-$(date +%Y%m%d-%H%M%S).jsonl"
```
Do not assume the test may safely move to Sleep from an obstructed or
unknown physical starting configuration. No production broker cutover.

## STOP qualification blocker from supplied 2026-10-11 logs

**Do not repeat physical STOP-during-motion or REARM tests on the arm yet.**
The supplied `stop-20261011-113611` motor trace + broker events show
that 24 ms HTTP STOP completion was followed by measured physical
elbow motion of 18.4° in approximately one second, even though no new
host-streamed motion commands were issued. The old `passed=true`
summary was a **false positive**. After the failed HOLD, extra physical
STOP retries could compound movement; the trial now forbids a blind
second position latch.

Before another hardware test, inspect the Feetech STOP implementation,
servo movement/goal/acceleration semantics, and the 1/1 physical-latch
profile against the 0/254 streamed profile. Use the recorded traces as
**evidence**, not a hardware safety acceptance. New software-only
regressions replay the trace's elbow drift and verify a faulted session,
revoked lease, HTTP error, and no automatic recovery. **No physical
STOP qualification or production cutover is authorized until this
failure is resolved with separately approved unloaded hardware tests.**

## PR #90 independent STOP and fault regression (open, unmerged)

Hardware-free software verification at the exact PR SHA:

```bash
ruff check .
python -m pytest --no-cov -q \
  tests/test_broker_stop_faults.py \
  tests/test_broker_stop_trial.py \
  tests/test_broker_sdk.py \
  tests/test_broker_session_lifecycle.py \
  tests/test_feetech_backend.py
python -m pytest --no-cov -q
```

The new suite uses simulated arms and an authenticated fake native
broker. It also checks SDK-specific MotionCancelledError, HardwareFaultError
and CommunicationError are returned as audited HTTP 409 errors rather
than escaping the broker worker; STOP success on real hardware requires
a positive torque-held readback, not merely a successful HTTP status. It asserts that late nominal SDK completion after a competing
STOP is audited/replied as failure; a generic failure in an attempted
STOP/HOLD latches a fault and revokes physical-trial motion authority;
a disconnected or faulted device cannot report completed STOP; and
the top-level arm always attempts tool/gripper STOP when arm HOLD
throws. Existing broker tests must retain STOP concurrency, queued
generation invalidation, HTTP profile denial and exact per-request
evidence. CI must never open the physical serial follower.

**Required attended local physical gate, separate from software CI:**

A reusable trusted-host procedure is now in
`examples/persistent_broker_stop_trial.py`. It never runs in CI or as an
OpenShell agent. **Start with the arm physically at Sleep**: the script
checks every measured arm joint against the calibrated, SDK-generated
Sleep target (5° tolerance) and refuses to auto-correct a mismatch. It
requires a calibrated saved Overhead pose meaningfully distinct from
Sleep (at least one joint differs by 10°), a human-issued lease,
physical TTY, and `ROUNDTRIP`, `START`, a **manual ENTER while moving**, and `STATIONARY` confirmations. The default 8 deg/s, 25 deg/s², 20 Hz are **conservative
test candidates**, not proven loaded-arm capabilities.

```bash
cd ~/AgenticForge/soarm101-motion-sdk
source .venv/bin/activate
soarm101 agent arm --minutes 30
python examples/persistent_broker_stop_trial.py \
  --port /dev/ttyACM1 --robot-id so101 \
  --output "$HOME/soarm-motion-tests/stop-$(date +%Y%m%d-%H%M%S).jsonl"
```

It preflights Overhead provenance **without requiring unrelated Left/Right
poses**, verifies measured starting Sleep, then completes and measures
**Overhead -> Sleep** using the trusted broker, recording both baseline
leg results and their actual joint errors. No initial Sleep motion is
commanded. Only after completing that return does it start a **new
Overhead movement** and displays a prompt to **press ENTER while the arm is moving**. Only that operator keystroke sends authenticated `POST /v1/stop`; if the arm finishes before ENTER the test fails rather than counting STOP. It captures the independent STOP response time and
verifies the in-flight reply is a rejection, authority is revoked,
and the SDK is no longer moving. It records the broker
events, motor trace and summary. After 1 second it requires an explicit
human observation of stationary posture before it can mark the trial
passed. A timeout, readback or HOLD fault is **not** STOP success.
**STOP leaves the arm between Sleep and Overhead and revokes the original
lease.** After `STATIONARY`, the operator can decline further movement,
or type `REARM` in the trusted host terminal to authorize a *separate*
joint-only Overhead movement. This requires a fresh verified HOLD,
calibration/robot identity checks, a new five-minute human lease and
normal broker validation. No cancelled trajectory is resumed. The
completed Overhead position is measured and the renewed authority is
cleared. A recovery failure cannot turn a failed overall trial into success.
Power cutoff must remain immediately accessible; discontinue on any
unexpected behavior. Additional manual visual/trace review is required
before merge regardless of script exit status.

Verify current branch SHA, calibration, measured start/end poses and
clearance, power cutoff, and that the arm has no payload. Use only the
trusted-host, loopback-only physical broker with the fixed trial
profile from `examples/persistent_broker_motion_trace.py` and an
explicit human `soarm101 agent arm` lease. Exercise a previously
validated safe pose transition with an independent authenticated
`POST /v1/stop` while **movement is visibly underway**. Verify:
the STOP is admitted while movement is in progress rather than queued;
no subsequent motion command executes *without a new explicit human
REARM*; the in-flight response is a rejection, never success; the human lease is revoked; the robot
settles safely; all five arm joint positions and gripper are inspected;
and no further movement occurs. Record monotonic STOP request/response
timestamps, physical observation, broker events, trace, failures and
held/fault state. Do not reuse a prior five-leg success as STOP proof.
Stop immediately on a communication, gripper, or HOLD error and inspect
the robot before any renewed arming.

This is a manual attended *qualification*, not a CLI flag added by #90.
Do not conduct this physical test in GitHub CI, unattended Codex, or
an OpenShell agent shell. Local Codex can help inspect and collect
traces on the trusted host with direct human control. Treat inability
to issue independent STOP, unclear motion state, no position readback,
or incomplete HOLD as a blocked merge gate for #90.

## PR #89 simulated ownership and lifecycle regression

Test on the exact PR #89 commit without connecting a real arm:

```bash
ruff check .
python -m pytest --no-cov -q \
  tests/test_broker_session_lifecycle.py \
  tests/test_broker_sdk.py tests/test_feetech_backend.py
python -m pytest --no-cov -q
```

The lifecycle suite uses a fake connected arm: verify at most one connect,
closed/faulted terminal states, hardware-failure lease revocation,
calibration-identity mismatch, STOP on a preexisting faulted/closing session,
shutdown waiting for active ordinary commands, and no ghost resumption.
Feetech tests use the safe `/dev/null` character-device lock to verify
that a partially opened port is closed before the advisory lock is
released, including a secondary cleanup error. They **never** instantiate
a real SO-ARM101 or command hardware.

For an attended **read-only real tty ownership check** on the operator's
actual workstation, first close the GUI, teleop and other port users,
confirm calibration and motor diagnostic health, keep robot power accessible,
and do not arm or enable torque.

In Terminal A, hold a diagnostic-only Feetech session (no motor commands):

```bash
cd ~/AgenticForge/soarm101-motion-sdk
source .venv/bin/activate
python - <<'PY'
from soarm101_motion.config import SOARM101Config
from soarm101_motion.hardware.feetech import FeetechBackend
config = SOARM101Config(
    port="/dev/ttyACM1", robot_id="so101", auto_enable_torque=False,
    configure_motors_on_connect=False, disable_torque_on_disconnect=False,
)
backend = FeetechBackend(config)
backend.connect()
try:
    print("READ-ONLY SERIAL OWNER ACTIVE; press Enter to release", flush=True)
    input()
finally:
    backend.disconnect()
PY
```

In Terminal B while A is holding the tty, run `soarm101 read --port
/dev/ttyACM1 --robot-id so101 --json`. Expect a device-ownership refusal,
not any movement or successful second serial connection. Release Terminal A
with Enter; rerun the same read command and expect ordinary read-only
diagnostics. This is a test of cooperative same-lock-namespace processes, not
a replacement for GUI/second-broker, out-of-process STOP, failure/crash,
or multi-user hardware qualification. Save console logs and exact SHA.

The human must separately validate that the current GUI, CLI and broker
cannot simultaneously acquire the real follower, that failed initialization
releases ownership, and that no motion resumes after reconnect. Preserve
console logs, exact SHA, calibration and tty identity. This is not permission
for automated hardware fault injection. Active-motion STOP/HOLD, native
camera and a measured 2 mm Cartesian trial remain separate PR gates.


## 2026-10-10 OpenShell/MCP broker 409 preflight regression (PR #88)

Observed physical-workstation error before Codex started:
`broker HTTP 409: default_joint_speed must not exceed its configured maximum
0.13962634015954636`. The operator-pinned 8 deg/s broker ceiling was
correctly propagated into the bounded CLI, but the CLI still constructed
`SOARM101Config` with the faster ordinary default of 0.45 rad/s. The 25 deg/s^2
joint acceleration, 10 mm/s Cartesian speed, and 40 mm/s^2 Cartesian acceleration
ceilings created analogous contradictions.

The CLI converts explicit absolute caps to SI and lowers only the corresponding
*internal defaults* where necessary. Tests exercise the actual read-only
`agent capabilities` CLI path with low caps and inspect the parsed simulated
`agent state` session configuration without connecting to any serial device:
`pytest -q --no-cov tests/test_cli.py tests/test_broker.py tests/test_agent_sandbox.py`.
This fixes preflight configuration; real OpenShell provider, camera, MCP tool,
and physical motion behavior still require separate local validation.


## 2026-10-10 physical broker 15/150 precision regression

User's first physical persistent-broker route failed *before opening hardware*
at `AgentMotionRates.validated()`: 15°/s was strictly greater than
the radians->degrees output 14.999999999999998°/s. The shared
`rate_within_ceiling()` comparator accepts only 4 ULPs of format
rounding and still rejects any meaningful speed increase. Test
`tests/test_broker.py::test_operator_rate_exactly_matches_roundtrip_human_limit`
and `tests/test_cli.py::test_cli_operator_rate_exactly_at_converted_ceiling`;
run full CI before supervised retry. A successful startup does not by itself
prove physical STOP/HOLD, port exclusivity, or all five motion legs.


## Persistent SDK broker five-pose physical experiment (PR #88)

The new `examples/persistent_broker_motion_trace.py` is an
**operator-only physical bench procedure** and must never run as part
of unit tests or GitHub CI. Automated CI uses a simulated SDK backend
behind the *actual loopback HTTP broker*, issues the five Sleep/Overhead/
Left/Right/Sleep POST requests, requires the same `broker_session.id`
and `sdk_connect_count == 1` each leg, and confirms the ordinary
CLI executor is never called. Additional regressions cover opt-in
access, locked-down capability profile, reject-external bind, invalid
`joint_only` request, human authority, STOP revocation and Linux tty
advisory lock.

Run `ruff check .` and
`python -m pytest --no-cov -q tests/test_broker_sdk.py
tests/test_broker.py tests/test_feetech_backend.py
tests/test_sdk_capabilities.py tests/test_poses.py` before the full suite.

The restricted physical trace CLI must reject non-TTY input and must not
accept `--yes`. `tests/test_broker_sdk.py` verifies the unattended launch
fails before starting the broker, writing output or opening a serial device.
The full robot trace remains an operator-only, human-confirmed hardware test.

Hardware handoff: verify a single real tty owner, calibrations, the
lease and powered HOLD/STOP, then run
`soarm101 agent arm --minutes 30` and
`python examples/persistent_broker_motion_trace.py
--port /dev/ttyACM1 --robot-id so101
--speed-deg-s 15 --acceleration-deg-s2 150
--command-frequency-hz 50 --pause-s 0.5
--output ~/soarm-motion-tests/persistent-broker-local.jsonl`.
Use a new unique output path. Operator must enter `RUN`.
Do not use `--include-gripper` initially because an earlier
almost-closed gripper triggered a transient overload status.
The physical profile remains a *candidate* until inspected by the
operator. Capture terminal output, `.summary.json`, motor trace
JSONL, broker events JSONL, physical STOP result, and any shaking
or tracking divergence. No Codex/OpenShell agent-controlled movement
is established by this experiment.


## Global trusted broker motion profile regression (PR #88)

Run `python -m pytest --no-cov -q tests/test_broker.py
tests/test_broker_sdk.py tests/test_mcp_server.py tests/test_robotctl.py
tests/test_agent_sandbox.py tests/test_poses.py`, then the complete
suite and `ruff check .`. Tests must confirm a single operator-selected
joint pacing limit is forwarded to saved poses, Sleep, Sleep-up and
single-joint jogs, not only jogs. Cartesian jogs must use mm/s and
mm/s² limits, and gripper portions must use independent raw Feetech
rate caps. MCP/robotctl may request lower rates, but rejected
above-cap/nonfinite/zero rates must issue no hardware command.
OpenShell's host process must pass the selected policy to the broker;
it must never be granted to an agent as a mutable privileged setting.
Check the agent's `robot_capabilities` projection for the selected
broker rate limits.

Local Codex handoff: test the exact PR SHA on the existing
`soarm101-motion-sdk` checkout with the above commands. No live
motion, broker switch-over, or service restart is part of automated
CI. Later run an attended 15°/s, 150°/s² joint-only route,
then a single small motion via a separately authorized Codex/MCP
profile, recording traces and STOP/HOLD. The native persistent SDK
path remains simulation-only pending its independent hardware gates.


## Gripper overload and joint-only Sleep regression (PR #88)

The 2026-10-10 local supervised route failed with `[ServoStatus] Overload`
reported by `so101_gripper`. A status bit from the servo is not evidence
of current joint load or a verified mechanical jam. The backend must **fail
closed** for motion and report the servo status, but STOP/HOLD must attempt
a live-position latch on every independently readable motor even if the tool
motor is unreadable. Incomplete HOLD must not be reported as success.

Run `python -m pytest --no-cov -q tests/test_feetech_backend.py tests/test_poses.py`
and the complete test suite. The joint-only experiment path is
`python examples/motion_quality_trace.py --joint-only ...`; it must
retain the default five-leg Sleep/Overhead/Left/Right/Sleep trajectory,
normal servo/fault checks, telemetry, and first-failure reporting, but
avoid the separately commanded Sleep gripper close. The default
`move_sleep()` behavior is unchanged. Before a physical retest,
inspect the gripper status bit and existing trace, and determine whether
the fault is still active. This is pending workstation/hardware validation.

## Sleep executable endpoint regression (PR #88)

Run `uv run pytest --no-cov tests/test_poses.py` plus the complete suite.
`Sleep` and `sleep_up` must select powered target angles strictly inside
calibrated effective joint limits (2° additional inset for ordinary ranges,
proportionally inside exceptionally narrow ranges). Their baseline wrist
orientation differences remain. Verify on a supervised unloaded arm only
after the independent joint-direction and STOP/HOLD gates; automated tests
must not invoke actual hardware motion. A Feetech control-bit reply warning
with verified readback is a separate communication diagnostic, not evidence
of a joint-limit failure.

## MCP virtualenv Landlock regression (PR #88)

The Codex workstation emitted `PermissionError: [Errno 13] Permission denied:
'/opt/soarm101-mcp/pyvenv.cfg'` during the **read-only**, pre-agent MCP
handshake. Run `python -m pytest --no-cov -q tests/test_agent_sandbox.py`;
`test_mcp_policy_allows_reading_only_its_isolated_venv` must verify the fixed
MCP venv is readable for both Hermes/Codex and both broker access profiles,
but is never writable through the filesystem policy or exposed in `robotctl`
mode. Physical OpenShell/virtualenv import must then be revalidated on the
operator workstation, with no arming or motor movement.

## Isolated MCP preflight diagnostics

Run `python -m pytest --no-cov -q tests/test_agent_sandbox.py` and check
that a failing mocked MCP preflight reports exit status and bounded stdout/stderr
with the per-run broker token redacted. A nonzero probe exit or missing readiness
marker must fail before the agent subprocess is launched. The real Codex image
and OpenShell host policy still require separate local validation; `sandbox doctor`
checking image existence does not prove MCP dependencies are present.

## One-command software validation for PR #88

On Ubuntu/Linux, copy and paste:

```bash
curl -fsSL https://raw.githubusercontent.com/AgenticForge-Labs/soarm101-motion-sdk/feature/broker-persistent-sdk-session/scripts/test_broker_pr88.sh -o /tmp/test_broker_pr88.sh && bash /tmp/test_broker_pr88.sh
```

The [repository test runner](scripts/test_broker_pr88.sh) clones or
fast-forwards a clean checkout under
`~/AgenticForge/soarm101-motion-sdk`, activates an isolated Python 3.12
environment for the script, and runs Ruff, focused broker tests, the
complete test suite, registry inspection and GUI simulation smoke checks.
It refuses to overwrite uncommitted changes or pull a divergent branch.
It never opens a real serial device or activates motor torque.

Afterward, run `cd ~/AgenticForge/soarm101-motion-sdk && source .venv/bin/activate`
to activate that environment in your current terminal.

## PR #88 simulated persistent broker contract (not physical validation)

Run `uv run ruff check .`, `uv run pytest --no-cov
tests/test_broker_sdk.py tests/test_broker.py tests/test_capability_profile.py`,
and the full `uv run pytest` suite. The native broker preview
`--sdk-simulation-preview` should preserve bearer authentication, pin a
read-only profile, reuse one simulated SDK session, and reject all POST
routes. Contract tests cover unknown arguments, missing/expired leases,
invalid joints, no automatic torque, and refusal to interpret model space
as measured physical clearance. Additional internal simulation-only HTTP
tests (not exposed by the broker CLI) verify bearer authentication, profile
rate intersection, STOP racing an in-flight fake joint move, a 409 error
rather than false success on cancellation, and observable failed HOLD
with audit evidence. STOP also cancels requests already queued behind the
broker dispatch lock and revokes the native lease, even on failed HOLD.
Fake native camera capture tests also verify exact image-byte SHA-256,
no disclosure of host capture paths or device nodes to broker clients,
and no success evidence when a trusted capture file is missing.
These tests do not validate servo-bus concurrency or real cameras.

**Not validated or enabled:** the real servo bus, exclusive ownership
across GUI/CLI/broker, interruptible HTTP STOP under a moving hardware SDK,
full native capture evidence, stale-geometry retry and model/physical jog
agreement. Before PR #88 can merge, these require fake-transport tests and
supervised unloaded hardware testing under `docs/physical-run.md`.
Software-only pytest never clears the physical gate.



## One-command PR #87 checkout and simulation validation

From an Ubuntu/Linux terminal, run this **single copy-and-paste line**:

```bash
curl -fsSL https://raw.githubusercontent.com/AgenticForge-Labs/soarm101-motion-sdk/feature/shared-sdk-capability-registry/scripts/test_registry_pr87.sh -o /tmp/test_registry_pr87.sh && bash /tmp/test_registry_pr87.sh
```

The source is tracked at [`scripts/test_registry_pr87.sh`](scripts/test_registry_pr87.sh).
The runner clones or fast-forward pulls branch
`feature/shared-sdk-capability-registry` into
`~/AgenticForge/soarm101-motion-sdk`; it refuses dirty worktrees,
unexpected remotes and divergent local branches. It then creates or updates
the isolated `.venv` (using `uv` when installed, otherwise Python
3.10–3.13 `venv`), activates that environment **for the test process**,
installs dev/GUI/simulation/MCP dependencies, and runs Ruff, compilation,
registry introspection, the focused and complete test suites, GUI offscreen
smoke test and CLI simulation. Logs remain under a reported temporary folder.

For an existing repository installed elsewhere, set
`SOARM101_CHECKOUT=/absolute/path` before the `bash` command.
To keep the environment active **after** the test, use:

```bash
cd ~/AgenticForge/soarm101-motion-sdk
source .venv/bin/activate
soarm101 sdk-capabilities --json
soarm101-gui --simulation
```

The scripted validation makes **no live hardware connections and does not
arm or move the robot**. Physical operation remains separately supervised,
with documented calibration, workspace and emergency power gates.
This script tests the open PR branch, not the current `main` release.

## Shared SDK capability registry — PR 1 (software-only)

Run `uv run ruff check .`, `uv run pytest --no-cov
tests/test_sdk_capabilities.py tests/test_cli.py`, and the complete normal
test suite. Confirm that `soarm101 sdk-capabilities --json` works without
any serial port, profile, torque, camera access, broker, or agent lease.
Tests must check unknown/extra/invalid payload rejection before SDK calls,
existing human CLI confirmation and JSON response parity, no automatic
torque enable, and retention of SDK dynamic safety guards.

This PR intentionally does not migrate broker subprocess execution.
Saved-pose listing/capture/replay, camera-profile readout and named-camera
capture have focused fake/simulation regression coverage. Verify pose
calibration provenance is checked before torque activation and again before
motion; camera capture must retain saved profile settings, device overrides,
output paths and --all semantics. Operator-only administration, serial
calibration, trajectories/sequences, and physical motion are not migrated or
validated by this PR.
Physical motion testing is not needed for pure registry introspection;
do not interpret simulation or fake arms as real-hardware validation.


## GUI test-session lifetime (PR #86 merge gate)

The CI matrix runs Python 3.10 and 3.12 with the optional PySide6 GUI
installed. GUI tests create numerous `QApplication`-owned widgets and worker
threads in headless offscreen mode. PR #86's Python 3.10 run completed 569
tests and Ruff successfully (80.71% coverage), then aborted with
`QObject: shared QObject was deleted directly` / `malloc_consolidate`
(exit 134). This is a CI failure despite pytest's pass count.

A test-only, session-scoped `QApplication` lifetime fixture was tried on
commit `14150c30b98ca7b9e50d871276a612a5fb9feb64`. It **did not fix**
the crash: 569 tests again passed before the same exit-134 failure. The
fixture was reverted; do not assume that Qt lifetime alone is the cause.

**Dependency regression lead:** comparing the last green `main` CI job
(Python 3.10, `113139291306`) with the failing PR job
(`113582596764`) shows that identical `uv sync --extra dev --extra gui`
resolved PySide6/Shiboken **6.11.2** versus **6.12.0**, respectively.
The failing run also downloaded new PySide6 WebEngine/PDF wheels.
Because `pyproject.toml` previously allowed `<7`, the resolution drifted
without a source code change. As a bounded experiment, `gui` now requires
`PySide6>=6.8,<6.12`. This is not proof of causality until the whole
CI process exits zero. Keep the cap only while evidence supports it;
a later controlled PySide upgrade requires independent GUI test coverage.

Before another speculative fix, local Codex should isolate GUI teardown
without changing the test definitions or ignoring process exit codes:

```bash
export QT_QPA_PLATFORM=offscreen
uv run pytest --no-cov tests/test_gui_*.py tests/test_camera_worker.py \
  tests/test_optional_gui_import.py
uv run pytest --no-cov tests/test_agent_control.py tests/test_motion_trace.py \
  tests/test_validated_linear_plan.py
uv run pytest
```

If GUI-only crashes, bisect GUI test modules and inspect native QObject
ownership/Qt thread cleanup. If only full suite crashes, identify its
cross-test interaction/order and compare environment/dependencies with
green `main`. A genuine fix must make the entire CI **process exit 0**
on 3.10 and 3.12. Do not suppress the abort or mark the affected tests
xfail merely to green the build.



## 100/1000 motion-envelope validation — 2026-10-06

The original absolute joint ceilings (1.0 rad/s, 5.0 rad/s^2) came from an early safety
hardening pass and had no documented SO-101/STS3215 hardware derivation. After the
40 deg/s, 250 deg/s^2 Sleep-vs-`sleep_up` rerun completed cleanly with the responsive
0/254 servo profile, the host characterization envelope was made explicit and human-facing:

```text
joint        100 deg/s, 1000 deg/s^2
TCP linear   100 mm/s, 1000 mm/s^2
TCP angular  100 deg/s, 1000 deg/s^2
```

Automated acceptance must prove that 80 deg/s, 500 deg/s^2 joint motion is inside the new
envelope and >100/>1000 requests are rejected. Broker tests must prove that the trusted-host
envelope is injected into bounded motion subprocesses while camera capture receives no
irrelevant motion flags.

The previously rejected **80 deg/s, 500 deg/s²** request was rerun on hardware on
2026-10-06 and completed successfully. The operator reported that motion was substantially
better/smoother than the earlier conservative runs. The servo remained on responsive 0/254
tracking; following-error, effort, fault, calibrated limits, workspace, timing, STOP/HOLD,
and settle guards remained unchanged.

The first **100 deg/s, 1000 deg/s²** attempt did not command hardware motion. It exposed a
floating-point boundary bug: `math.radians(1000.0)` differed from the configured
1000-deg/s² SI ceiling by only a few ULPs, so the strict `resolved > maximum` comparison
rejected two values that both formatted as 17.4533 rad/s². Regression coverage now requires
exact advertised 100/1000 values to pass while >100/>1000 requests still fail.

The next supervised characterization point is therefore:

```bash
bash scripts/run_sleep_posture_comparison.sh 100 1000
```

Do not describe 100/1000 as physically validated until that run completes cleanly under the
unchanged runtime guards.


## Passive agent-jog motion/HOLD trace

Opt-in diagnostics were added after a user observed model-space downward
creep during a sequence of 2 mm horizontal Cartesian commands. Endpoint IK
returned an internally consistent horizontal solution, but that alone could
not separate incorrect path commands, servo tracking, HOLD re-latching, and
subsequent gravity sag.

`soarm101 agent jog --trace-file /tmp/soarm101-jog-001.jsonl`
reuses `PassiveBackendTrace` for the actual execution, including internal
`_write_raw_positions` writes used by Feetech STOP/HOLD. The trace records
preflight intent, existing command and feedback traffic, pre-HOLD settled
joint state, precise hold markers and raw latch writes, plus encoder positions
immediately and 2 s after HOLD. No new high-rate feedback polling is added
during a trajectory. JSONL events carry runtime robot/calibration provenance.

`soarm101 agent trace-summary /tmp/soarm101-jog-001.jsonl`
reads the trace **offline** and reports model-space commanded/observed Z
changes and raw HOLD-goal deltas. It makes no physical-clearance claim.

Hardware-free focused suite:

```bash
pytest --no-cov tests/test_motion_trace.py tests/test_agent_control.py
ruff check src/soarm101_motion/cli/main.py src/soarm101_motion/motion/trace.py \
  tests/test_motion_trace.py tests/test_agent_control.py
```

For hardware: run only on the checked-out commit, with a matching calibrated
follower and human `agent arm` lease, clear table and no payload. Stop the
broker/GUI before using the direct CLI. Begin with an unloaded 2 mm world-X
jog at historic 10 mm/s and 40 mm/s² and an independent side view of
the gripper/table. Never intentionally provoke an unsafe sag or following
error. Report the full trace and offline summary; determine whether the
descent occurs in the planned command, encoder feedback, raw HOLD latch,
or the following 2 seconds before increasing rates.

## Stale Cartesian preflight on low-cost servos

The live Codex MCP run on 2026-10-08 produced three broker HTTP 409
rejections reporting `robot joints changed after Cartesian path
validation; retry the move` (request IDs starting `134dabe`,
`dbe89fd`, and `4e4cf4f`). This was a pre-execution stale-plan
check, **not** evidence of following error, motor overload or collision.

The underlying managed controller keeps its narrow cached-plan
start tolerance. The facade now retries the *whole* planning and
workspace-validation sequence at most twice on a typed
`StaleCartesianPlanError`, preserving fail-closed behavior for
persistent drift or other safety failures. Validate the focused
hardware-free regressions:

```bash
pytest --no-cov tests/test_validated_linear_plan.py
```

For physical validation: keep the arm unloaded, workspace clear, and
the previously calibrated setup, begin with conservative 2–5 mm
jogs at existing requested speeds, and verify that unexpected
start drift either causes safe revalidation or a clean refusal with
no intermediate motion. Do not use a real object grasp or faster
rates to test this change. Local servo behavior remains
unvalidated until this check is performed.

## Trusted broker requested-motion rate configuration

Hardware-free tests cover broker defaults, custom joint/Cartesian requested
rates, positive/finite/envelope checks before subprocess execution,
propagation only into bounded joint/jog commands, and CLI parser/rate
validation without opening the follower. Run:

```bash
pytest tests/test_broker.py tests/test_agent_control.py
```

Before an operator selects higher real-hardware rates, investigate the
prior Cartesian physical-height drift and repeated HTTP 409 errors,
validate motion settle and trajectory behavior at existing defaults,
and perform supervised incremental speed characterization. A
successful startup or test is not hardware validation.

## Broker capability-profile testing

Run `pytest tests/test_capability_profile.py tests/test_broker.py
tests/test_mcp_server.py` without any robot or credentials. Validate that
profile JSON rejects unknown capabilities, invalid limits and cameras; that
hidden tool routes, aliases, or oversized movements are rejected at the
authenticated broker even when called outside MCP; and that the stdio MCP
tool list reflects the same trusted-host profile. This is separate from
local supervised physical validation.

The `mcp-profile-supervised-manipulation.json` example must parse and
advertise both cameras, joint control limited to 10 degrees per request,
Cartesian jogs, saved poses, Sleep, gripper and STOP/HOLD. Tests should
confirm the optional model-frame jog cap is absent so the unchanged
bounded agent CLI's calibrated *physical* 50 mm/10 mm start-height guard
remains authoritative. This must not be interpreted as validating 50 mm
movements or higher speed on real hardware; investigate previously observed
height drift and HTTP 409 rejections before increasing speed or step size
during live object approach.

## MCP guidance contract testing

Run `pytest tests/test_mcp_guidance.py tests/test_mcp_server.py tests/test_capability_profile.py` without hardware. Confirm server initialization exposes MCP-first live
robot instructions, tool descriptions direct camera/state use through the
broker, resources/prompts preserve calibrated-coordinate and evidence rules,
and no guidance changes the active MCP tool allowlist. A separate read-only
Codex workstation check should ask for both cameras **without** saying
"use MCP" and confirm the agent directly selects `soarm101.capture_camera`
for each permitted camera instead of host `soarm101 camera` commands. This
host/model behavior is not established by the deterministic tests.

## MCP OpenShell integration tests

Run `pytest tests/test_agent_sandbox.py tests/test_mcp_server.py
tests/test_capability_profile.py` using fake OpenShell and broker fixtures.
Checks cover Codex/Hermes per-run MCP config, explicit environment-variable
inheritance without literal credentials, client-only module uploads, profile
pinning, read-only broker and network reductions, MCP image evidence retention
and SHA mismatch rejection, and unchanged robotctl defaults.

Actual image builds, isolated provider authentication, native MCP tool/image
visibility, and hardware motion remain separate local Codex/supervised
validation gates. Do not infer physical reliability from these tests.

## Optional MCP agent facade

Install the optional MCP extra with `pip install -e '.[mcp]'` and run
`pytest tests/test_mcp_server.py tests/test_broker.py tests/test_robotctl.py`.
The MCP integration tests use the SDK's in-process MCP test client and a fake
broker request function; they must never command physical hardware. Confirm
`tools/list` exposes only bounded routes, typed coordinates/frames reject invalid
inputs, camera pixels are SHA-verified and host paths are excluded, and broker
authority errors propagate back as tool failures. See [docs/agent-mcp.md](docs/agent-mcp.md)
for separate read-only workstation validation.

## Self-contained agent sandbox

Automated tests remain hardware-free and cover packaged asset availability, OpenShell policy
allowlists, CLI wiring, human-authority preflight, minimal task/skill/client uploads, auth
selection, provenance, and sandbox/broker cleanup.

Before any physical autonomous run, validate each selected adapter through its read-only path:

```bash
soarm101 agent sandbox doctor --agent hermes
soarm101 agent sandbox run --agent hermes --read-only

soarm101 agent sandbox doctor --agent codex --auth installed
soarm101 agent sandbox run --agent codex --auth installed --read-only
```

The packaged read-only task uses only capabilities/state/camera capture. It must work without
`soarm101 agent arm`; an absent authority lease blocks motion but must not stop read-only
observation. The effective broker policy must contain no pose/joint/jog/gripper/Sleep/STOP
routes. The SDK then requires post-handoff broker evidence for capabilities, state, and one
successful capture of every configured camera, verifies downloaded image SHA-256 values
against trusted capture events, rejects any broker-observed motion action, and writes
`read-only-validation.json`. A zero harness exit without that evidence is a failed gate.
Also verify the sandbox has no Motion SDK checkout, serial/camera devices, calibration files,
Docker socket, or unrelated host credentials, and that the sandbox is deleted after the run.
Only after that gate should a supervised motion run begin.

## Bounded agent CLI

Automated tests cover authority expiry/identity matching, non-interactive arming rejection,
agent-only pose filtering, simulation motion commands, and the measured-height jog policy.
Before physical agent use, run a supervised validation from a clear workspace:

```bash
soarm101 agent arm --minutes 15
soarm101 agent capabilities
soarm101 agent state
soarm101 agent poses
soarm101 agent cameras
soarm101 agent go-pose agent_start_overhead
soarm101 agent capture overhead
soarm101 agent capture wrist
```

Confirm every successful motion remains torque-held. Confirm missing/expired authority
rejects motion. After reaching `agent_start_overhead`, run `soarm101 agent sleep` and
confirm the calibrated fold is accepted without an apparent equal-to-limit rejection; the
arm must remain inside the configured 1° measured-stop inset and hold after completion.
From measured heights above and below 100 mm, validate that physical jog
requests over 50 mm / 10 mm respectively are rejected before motion and that targets entering
the 10 mm calibrated ground-plane safety margin are rejected. Begin with much smaller
supervised jogs than the policy maxima. STOP/HOLD must remain available without authority.
End the session by disarming authority, then use human-confirmed `soarm101 relax` only when
physically safe.

### Bounded-agent physical validation record — 2026-10-03

Validation began from PR #76 head `2d220f56d30a9d5d8f222faec4b72f4d5b5e181d`
with the target workstation's saved follower, calibration, workspace calibration, and named
camera profiles. The focused agent/CLI tests passed 27/27; the repository's normal full CI was
already green at that head.

- `agent state` resolved `so101` and the active calibration/workspace identity correctly.
- Fresh `overhead` and `wrist` captures succeeded through their saved stable device paths.
- Human-interactive one-minute arming enabled/held without a visible startup jump. Expiry
  changed authority to unarmed, and a subsequent `agent go-pose` failed closed before motion.
- After re-arming, `agent go-pose agent_start_overhead` completed from the folded/Sleep-like
  start and remained torque-held after process exit. Transit was visibly shaky while moving,
  but the arm became stable once it reached the target. Treat joint-transit shake as a
  separate motion-quality follow-up rather than a hold failure.
- At about 127 mm measured physical height, a 55 mm requested jog was rejected against the
  50 mm ceiling and a roughly 3 mm jog completed and held.
- After conservative downward steps, at about 97.5 mm measured physical height an 11 mm
  requested jog was rejected against the 10 mm ceiling and a roughly 3 mm jog completed and
  held.
- Several low-height jogs finished a few millimeters from the workspace-predicted height while
  still satisfying the SDK's joint completion tolerance. The agent floor rule therefore keeps
  a 10 mm planned-target margin above the calibrated ground plane; do not use the command
  bounds as achieved-position metrology.
- After `agent disarm`, `agent stop` remained available and held the measured pose.
- `soarm101 relax` kept torque enabled until explicit ENTER confirmation, then relaxed the
  follower.
- The real arm was deliberately not driven near the floor merely to exercise the floor guard.
  Automated policy tests cover rejection of targets entering the configured margin.
### Measured folded-start workspace regression — 2026-10-04

Post-study exact replay initially failed before motion while the follower was in its normal
folded/resting state. The generic centerline model reported 0.021 m self-clearance versus the
0.025 m heuristic at workspace path sample 0. This was a planning false positive at the already
occupied measured start, not a newly commanded collision.

The reusable joint-space validator now treats a measured start inside the coarse self-clearance
envelope as admissible only while the path does not worsen that pre-existing modeled clearance
(with 0.5 mm numerical tolerance). The motion does not have to fully leave the generic envelope
to be an improving/neutral move. If the path reaches normal clearance, strict checking resumes
immediately and re-entry is rejected. Floor, base keep-out, TCP reach, calibrated joint limits,
rate/acceleration, following-error, effort, fault, communication, and settle protections remain
unchanged.

Regression tests cover: improving-but-still-inside motion, deeper-fold rejection, strict
re-entry rejection after clearing the envelope, and preservation of the floor guard.

### Exact teleop replay physical result — 2026-10-04

The post-study exact teleop replay completed successfully on the physical follower after the
replay was corrected to reuse the original GUI teleop safety settings. The operator could
clearly recognize the replay as the recorded teleop motion because its speed varied naturally,
and reported that it felt **less mechanical and smoother** than the generated programmed route.

This is the strongest physical discriminator in the low-speed investigation so far:

- live teleop is smooth enough to serve as the reference behavior;
- deterministic replay of the exact accepted teleop arm-command sequence also preserves that
  less-mechanical character;
- programmed 20 Hz versus 50 Hz motion showed no large subjective difference;
- the generated route remains especially shaky while folding back into Sleep.

Therefore frequent position streaming and 20 Hz transport are not sufficient explanations for
the shake. The leading hypothesis is now the **generated joint-command profile itself**:
per-joint discrete velocity/acceleration structure, long stretches of very small encoder steps,
stop/start behavior as joints enter low-speed portions of the synchronized path, and how those
profiles interact with gravity/backlash/static friction in the folded Sleep geometry.

Before another control change, compare the exact replay trace against both programmed traces
quantitatively: per-joint raw encoder increment distributions, commanded velocity and
acceleration, zero/near-zero runs, sign changes, command-versus-actual lag, and the approach to
Sleep. Do not treat cadence tuning as the primary next intervention unless the trace comparison
reveals a hidden timing effect.

### Exact-replay configuration mismatch — 2026-10-04

The first post-study exact replay preflight rejected original teleop sample 117 as
343.77 deg/s^2 against a 286.48 deg/s^2 ceiling even though the original live teleop had
accepted that command. The captured sample numbers were contiguous, ruling out the suspected
missing-frame explanation.

Root cause: the GUI follower is intentionally constructed with a teleop acceleration ceiling
of 6.0 rad/s^2 (343.77 deg/s^2), while the standalone replay had constructed a default SDK
configuration whose stream acceleration ceiling was 5.0 rad/s^2 (286.48 deg/s^2). The replay
was therefore not reproducing the original control contract.

Exact post-study replay now loads the original copied GUI session's `teleop_settings` event
and reuses its recorded joint-speed ceiling, joint-acceleration ceiling, maximum command step,
and following-error limit. The replay does not raise or infer limits: it reproduces the safety
settings under which the commands were originally accepted. Missing/invalid recorded settings
fail closed. Regression tests cover settings recovery.

### Sleep versus sleep2 — primary geometry check

The simplest operator-driven geometry experiment is now the preferred next test:

```bash
bash scripts/run_sleep2_comparison.sh 24 150
```

Before invoking the script, physically place the arm in the alternate resting pose you want
to evaluate. The Python program captures the **current measured pose before issuing any arm
motion or torque-latch command**, validates its calibrated joint coordinates, and persists it
as the named pose `sleep2` plus an experiment-local `sleep2.json`.

It then runs a direct A/B from the same RIGHT saved pose:

1. **A — canonical Sleep:** RIGHT -> built-in calibration-relative Sleep.
2. **B — sleep2:** RIGHT -> the recorded operator-selected pose.

The sleep2 route uses the same narrow Sleep-family exception for the generic coarse
self-clearance heuristic while retaining calibrated joint limits, floor, TCP reach, base
keepout, and normal streamed runtime guards. This intentionally avoids wrist-specific
teaching logic or inferred intermediate geometry; the operator chooses the complete alternate
resting configuration and the test compares that exact configuration against canonical Sleep.

The earlier manual-wrist diagnostic remains available for deeper follow-up, but sleep2 is the
preferred next test because it isolates the configuration choice with much less machinery.


Physical A/B follow-up on 2026-10-05 found the operator-selected `sleep2` configuration
**substantially smoother** than canonical historical Sleep, confirming a large geometry
component to the rocking. The chosen wrist orientation was on the opposite side of the
historical lower-limit wrist fold, motivating a portable calibration-relative default rather
than persisting one machine's taught coordinates.

The production semantic now defines default Sleep `wrist_flex` as:

```text
lower + 0.75 * (upper - lower)
= upper - 0.25 * (upper - lower)
```


Confirm the promoted semantic directly on hardware with:

```bash
bash scripts/run_sleep_posture_comparison.sh 24 150
```

This performs only two conditions from the same saved RIGHT pose: A is RIGHT -> default
Sleep and B is RIGHT -> `sleep_up`. The script prints both derived calibrated wrist-flex
targets before motion and packages the passive trace and summary for comparison.


First confirmation attempt on 2026-10-05 derived default Sleep wrist flex at **+51.46°** and
historical `sleep_up` at **-102.91°**, while all other Sleep joints were identical. A
(RIGHT -> default Sleep) completed. The subsequent reset from default Sleep back to RIGHT
timed out at the normal 5 s settle boundary with a worst `elbow_flex` error of 0.0261 rad
(~1.50°), just outside the configured 0.025 rad tolerance; hardware reported the elbow
stationary with no fault. B therefore did not run.


A later 40 deg/s, 250 deg/s² attempt exposed a separate actuator-profile mismatch before A:
the RIGHT reset tripped the normal 0.300 rad following-error guard on `wrist_flex` at
0.305 rad. The host trajectory had advanced to -0.556 rad while measured wrist flex was
-0.861 rad, so the physical joint was ~17.5° behind the host target. The cause was that
ordinary streamed joint moves still used the backend's fixed Feetech profile
(`speed_raw=250`, `acceleration_raw=20`) even when the host planner was asked to run much
faster.

The fix does **not** raise or disable following-error. Default streamed planned joint motion
now uses the same responsive Feetech profile as live teleoperation:
`speed_raw=0` (unrestricted/max) and `acceleration_raw=254`, unless a low-level caller
explicitly supplies servo-profile overrides. The validated host trajectory continues to own
joint speed and acceleration, while calibrated limits, following-error, unexpected-direction,
effort/fault, timing, STOP/HOLD, and settle checks remain unchanged. Re-run the 40/250
Sleep-vs-sleep_up comparison to determine whether the faster host trajectory is physically
smoother when the servos are no longer artificially throttled.


Physical rerun on 2026-10-06 at **40 deg/s, 250 deg/s²** completed the full
Sleep-vs-`sleep_up` comparison with the responsive 0/254 servo profile and no following-error
trip. This confirms the earlier 40/250 wrist lag was caused by the redundant slow actuator
profile rather than the requested host dynamics alone.

An earlier **80 deg/s, 500 deg/s²** request was rejected before motion by the then-current
1.0-rad/s / 5.0-rad/s² host ceiling. That historical rejection motivated the explicit
100/1000 characterization envelope. After the envelope change, the 80/500 hardware rerun on
2026-10-06 completed successfully and was judged substantially smoother by the operator.

The A/B harness now retries RIGHT once **only** after a `MotionTimeoutError` whose measured
worst joint error is within twice the ordinary joint-position tolerance. The retry reissues
the same fully guarded saved-pose move and must satisfy the normal settle check; no motion
tolerance or safety acceptance criterion is relaxed.

using the active executable calibrated range. All other Sleep joint selectors remain
unchanged. The previous fully folded wrist-at-lower-limit posture is retained as
`sleep_up`. Existing scripts using `move_sleep()` therefore exercise the new default
automatically; `move_sleep_up()` is the explicit historical override. The local `sleep2`
capture is evidence for this design decision, not a runtime source of truth.

### Sleep geometry and fold-order comparison

The 24 deg/s runs were subjectively somewhat smoother overall than 8 deg/s, and increasing
acceleration further did not create a large additional improvement. Severe rocking still
returned as the arm slowed into canonical Sleep, making folded geometry/load an important
remaining hypothesis.

The first automatic neutral-wrist experiment proved too indirect: neutralizing wrist flex
while keeping canonical Sleep shoulder/elbow targets still entered the same 0.021 m coarse
self-clearance state. The experiment now uses an operator-taught wrist angle instead of
guessing one.

Run:

```bash
bash scripts/run_sleep_geometry_comparison.sh streamed 24 150
```

The supervised workflow is:

1. Move into canonical Sleep and hold.
2. Relax **only `wrist_flex`** while shoulder pan/lift, elbow, wrist roll, and gripper stay
   torque-held.
3. The operator hand-places the wrist and presses ENTER.
4. The measured wrist angle must lie inside the executable calibrated wrist range, and held
   non-wrist joints must not have drifted beyond the normal joint-position tolerance.
5. The backend latches the freshly measured wrist position before re-enabling that motor.
   The resulting full pose is saved as `motion_test_sleep_wrist` and in the experiment
   folder as `taught-sleep-wrist.json`.
6. Compare:
   - **A direct canonical:** RIGHT -> canonical Sleep.
   - **B taught wrist:** RIGHT -> canonical folded arm geometry with the taught wrist angle.
   - **C staged wrist:** RIGHT -> taught-wrist Sleep, then move only wrist flex into canonical
     Sleep.

B/C reuse the same narrow rationale as canonical Sleep: only the generic coarse
self-clearance heuristic is omitted. Before powered motion the complete joint path is still
validated for calibrated joint limits, floor, TCP reach, and base keepout. Runtime
rate/acceleration/following-error/effort/fault/communication/settle guards remain active.
This diagnostic is intentionally streamed-only because asynchronous final-target motion would
require a different measured-path treatment inside the known folded self-clearance state.

The key observation is whether B is calmer than A, and whether rocking appears primarily in
C's final wrist-only fold. That directly tests wrist orientation/order while keeping the rest
of the folded geometry constant.


Manual-teach attempt on 2026-10-05 successfully relaxed only `wrist_flex` and captured the
operator-positioned wrist, but stopped before save/retest because the diagnostic computed
`max(other_drift)` on a dictionary, yielding a joint-name string rather than the largest
numeric drift. The failure occurred before the taught pose was persisted. The cleanup path
re-enabled `wrist_flex` and re-held the arm. The diagnostic now uses the shared
`maximum_joint_drift(..., exclude=("wrist_flex",))` helper, with regression coverage proving
the taught wrist can move substantially while the numeric maximum drift of the still-held
joints is evaluated correctly.

### Three-times-speed streamed versus final-target comparison

After the 8 deg/s, 25 deg/s^2 comparison showed substantial rocking in both execution
strategies, repeat the same route at 3x requested dynamics:

```bash
bash scripts/run_joint_execution_comparison.sh both 24 75
```

This keeps the route, saved poses, planning cadence, safety checks, and execution-mode
comparison unchanged while increasing requested joint speed from 8 to 24 deg/s and requested
joint acceleration from 25 to 75 deg/s^2. These remain below the configured host-side ceilings
of approximately 57.3 deg/s and 286.5 deg/s^2. Compare overall rocking and especially the
deceleration/fold into Sleep. Keep both traces and the generated archive.

### Streamed versus final-target joint execution — experimental

After the exact teleop replay established that deterministic streamed commands can retain the
less-mechanical teleop character, keep the existing streamed implementation and test a second
joint-only execution strategy rather than replacing it prematurely.

Run:

```bash
bash scripts/run_joint_execution_comparison.sh
```

The runner updates the diagnostic branch and executes the same
Sleep -> Overhead -> Left -> Right -> Sleep route twice at 8 deg/s and 25 deg/s^2:

1. **streamed** — the existing validated host trajectory sends intermediate joint goals.
2. **final_target** — the same joint plan is built/validated, then exactly one endpoint is
   written per leg with per-joint servo speed limits derived from the planned duration.

Both runs produce passive JSONL traces in one timestamped directory. Compare visible shake,
especially Right -> Sleep, plus command count, effective raw servo speeds, feedback progression,
joint coordination, and final settle. The final-target run is deliberately joint-space only;
do not infer anything about Cartesian `move_linear()` from this experiment.

The one-shot implementation remains experimental until supervised hardware validation shows
whether it improves motion quality without introducing timeout or safety regressions. The
default API behavior remains `streamed`.

First physical A/B attempt on 2026-10-05: the streamed route completed. In `final_target`,
the initial Sleep leg completed, but Sleep -> Overhead was stopped/held by the experimental
cross-joint phase guard when wrist flex differed from median joint progress by 0.308 rad
against the reused 0.300 rad following-error threshold. That rule was judged invalid for the
one-shot contract: independent servos can legitimately advance at different rates while still
moving in the commanded direction and remaining inside their validated start-to-target joint
corridors. The phase guard was removed; per-joint corridor/overshoot, reverse-motion,
hardware/effort fault, cancellation, timeout, and final-settle checks remain. Because
independent servo progress can produce intermediate configurations that differ from the
synchronized host plan, full-workspace/saved-pose final-target execution now also validates
the accumulated measured physical path on every monitor cycle using the existing
measured-start workspace policy. Use
`bash scripts/run_joint_execution_comparison.sh final_target` to continue the physical test
without repeating the streamed baseline.


Corrected physical rerun on 2026-10-05 completed the entire
Sleep -> Overhead -> Left -> Right -> Sleep route in `final_target` mode with no safety
trip, but the operator still observed substantial rocking. This rules out repeated host
micro-waypoint writes as the primary cause of the motion-quality problem. Keep both
`streamed` and `final_target` implementations for now: they are useful diagnostic
execution strategies, but neither is yet demonstrated to solve the low-speed shake.

The strongest remaining contrast is now **teleop-derived command shape versus programmed
low-speed motion**, not streamed versus one-shot transport. Exact teleop-command replay was
subjectively less mechanical, while both generated streamed motion and generated final-target
motion rocked, especially around the folded Sleep configuration. Before another architecture
change, inspect the recorded teleop and final-target traces for actual per-joint velocity,
raw goal-speed/profile values, low-speed dwell, load-dependent lag, and the final approach to
Sleep.

### Teleop versus programmed-motion trace protocol

Before changing PID, command cadence, or trajectory shape again, run the guided comparison
from the exact diagnostic branch/commit:

```bash
bash scripts/run_motion_quality_study.sh
```

The runner walks the operator through four conditions while preserving the normal hardware
guards:

1. **Live teleop reference.** The runner launches the GUI. Link the leader/follower, place
   the follower at the folded Sleep-like start, then mark the interval in the terminal and
   teleoperate slowly through Sleep -> Overhead -> Left -> Right -> Sleep. Stop teleoperation
   before marking the end. The runner copies the GUI session log and extracts only the marked
   `teleop_frame` interval.
2. **Exact accepted teleop-command replay.** Unless explicitly skipped, the runner replays the
   marked arm-joint command sequence through the existing guarded joint-stream primitive at
   the recorded nominal cadence/timing. If the follower is too far from the first recorded
   sample for a legal stream start, replay is skipped rather than bypassing the step guard.
   The gripper is intentionally omitted so this condition isolates arm-joint behavior.
3. **Programmed 50 Hz route.** The saved-pose route runs automatically at 8 deg/s and
   25 deg/s^2 using the current planned-motion 50 Hz cadence. There are no per-leg prompts.
4. **Programmed 20 Hz route.** The identical route, speed, and acceleration run automatically
   at a teleop-like 20 Hz cadence.

The passive program/replay tracer wraps only backend calls the SDK already makes; it does
not add motion-time hardware reads. Detailed GUI teleop logging records leader joints,
desired/limited command, command velocity, follower actual joints, following error, sample
interval/age, processing time, raw encoder command/actual values, and the effective 0/254
teleop servo profile. The program traces record outgoing joint commands, raw encoder goals,
effective servo parameters, natural feedback reads, TCP positions, hardware-state checks,
and route markers.

Compare command intervals, raw encoder increments, command-versus-feedback lag, repeated
raw targets, low-speed/deceleration regions, and folded versus extended geometry. The
experiment distinguishes:

- live teleop smoothness versus deterministic replay of the same accepted commands;
- teleop-derived commands versus minimum-jerk generated commands; and
- 20 Hz versus 50 Hz host cadence for the same generated route.

Treat current/load as a second-pass diagnostic if the kinematic/timing evidence is
insufficient, because extra effort polling can itself perturb serial timing. If the guided run's exact replay is skipped because the current follower pose is too far from the first captured command, finish conditions C/D and then run `bash scripts/run_motion_quality_replay.sh`. The post-study replay pre-positions to the first captured arm pose using ordinary guarded joint motion with full workspace checks, verifies measured arrival, preflights every captured command against active joint/step/speed/acceleration limits, and only then starts guarded exact streaming. Its trace, summary, and safety-refusal outcome are appended to the same study folder/archive.

A
single-final-target joint move remains a later discriminator for saved-pose motion; do not
generalize that experiment to Cartesian `move_linear()`, where a single joint endpoint
cannot guarantee the requested straight TCP path.

### 20 Hz versus 50 Hz physical comparison — 2026-10-04

On the physical follower, the same 8 deg/s saved-pose route did **not** show a large
subjective smoothness difference between 50 Hz and 20 Hz host command cadence. The dominant
visible failure remained strong shake/jitter while folding back into Sleep. This weakens
host update frequency by itself as the primary cause.

The configuration dependence is now the stronger clue: folded/Sleep motion is consistently
worse than the more extended overhead/left/right portions. Prior tests also showed faster
motion is better overall, but shake returns during deceleration. The remaining hypotheses
should therefore emphasize joint-specific low-speed behavior under changing gravity/load,
backlash/static friction near the folded configuration, and whether the host planner gives
some joints very small/stop-start discrete motions while another joint sets the overall
trajectory duration.

The next discriminator is the post-study exact teleop-command replay. If the exact accepted
teleop sequence remains smooth through the same return-to-Sleep fold, the hardware stream
path is capable of that motion and the generated planned trajectory/coordination is implicated.
If exact replay also becomes shaky in the fold, inspect load/configuration-dependent servo
behavior and live-teleop-specific differences before changing the planner.

### Slow-speed motion diagnostic record — 2026-10-04

A supervised saved-pose A/B/C comparison on current `main` used the same host-planned joint
path with three servo write strategies: the ordinary `250/20` profile, `0/254`, and
`Goal_Position`-only writes. All three looked smoother through the main transit than the
previously troublesome motion, but visible roughness concentrated as the arm slowed near
the endpoint. The `Goal_Position`-only condition later tripped the existing wrist-flex
following-error guard at 0.332 rad against the 0.300 rad limit during return, so it is not
adopted as a production transport change.

This points the next experiment at actuator-resolution behavior rather than another IK or
PID change. On the quantized-target branch, repeat the same safe saved-pose motion and compare
slow-tail smoothness against `main`. Confirm that intermediate writes with unchanged encoder
targets are suppressed, the host cadence/deadlines continue unchanged, the exact final
planned sample is still written, and following-error/fault/effort/settle behavior is unchanged.
Record whether the visible roughness improves specifically during deceleration.

## Testing roadmap

This file is the handoff checklist for physical testing. Implementation can continue in
simulation before any of these steps are run. Work through the sections in order when
you are ready to put the real SO-ARM101 on the bench.

## First physical bench session — stop gates

Use `docs/physical-run.md` as the short bench card. For the first powered session, stop
after any failed gate rather than continuing into later capabilities.

1. Discover/identify the follower and verify voltage, six motor IDs/models, diagnostics,
   and clean status with torque OFF.
2. Calibrate the follower through complete mechanical-stop sweeps. Record its displayed
   calibration ID and verify both the current calibration file and immutable history copy
   exist.
3. Disconnect the setup session and reconnect normally. A factory-range/uncalibrated
   session must now refuse torque enable before any goal or torque write.
4. Run the CLI 2° smoke test on **one joint at a time**, beginning with shoulder_pan.
   Confirm physical sign, clean return, STOP/power accessibility, and no fault/effort trip
   before proceeding to the next joint.
5. After all five pose joints pass, test the gripper from near mid travel with small
   normalized changes around 0.5. Do not make the first gripper command full open/close.
6. Characterize unloaded effort/current at the already-validated slow motions.
7. Only then save and test Home/Rest. Home/Rest replay is calibration-bound and now moves
   the arm first, then commands the saved gripper position.
8. Do **not** test Cartesian motion, recorded-trajectory replay, sequences, primitives, or
   leader→follower teleoperation during the first foundational run unless every preceding
   gate has passed.

### Follower position-hold authority

Before repeating gravity-loaded Cartesian tests after upgrading from a configuration that
used P=16, apply the current recommended motor settings once:

```bash
soarm101 configure --port /dev/ttyACM1 --robot-id so101
```

This does not redo mechanical-stop calibration. It disables torque while writing the
recommended servo configuration and preserves the existing saved calibration. Confirm
the arm is supported/clear while configuration runs.

The current recommended STS3215 position P coefficient is 32 (factory default). A motion
that reaches most of a target but remains several degrees short, especially while lifting
against gravity, must not be "fixed" only by increasing the settle timeout. First verify
position-hold authority, supply voltage, faults, and effort/current behavior.

On a paper-motion error, verify the script holds the current position and waits for an
explicit operator release instead of immediately relaxing.

### Cartesian servo-profile comparison with teleoperation

The SDK-wide planned-motion default remains 50 Hz. The supervised paper hardware
validation intentionally configures `move_linear()` at 20 Hz to match the known-smooth
teleoperation host cadence while retaining the same trajectory limits and guards. Servo
writes should use the same responsive Feetech profile as teleoperation:

```text
speed_raw = 0
acceleration_raw = 254
```

Do not reintroduce the generic 250/20 servo profile on every Cartesian waypoint; that
layers a second slow motor trajectory under the host trajectory and can appear as
lag/catch-up shaking.

On hardware, compare the same broad workspace motion with smooth 20 Hz teleoperation and
a 20 mm/s, 100 mm/s² paper replay. The supervised paper script uses a 20 Hz host command cadence and 1 mm Cartesian
planning-density bound. Every emitted command sample is a direct sequential-IK solution
of the Cartesian path. Planned Cartesian execution must use the same Feetech servo-side
profile as smooth teleoperation: `speed_raw=0`, `acceleration_raw=254`. The host
trajectory owns speed/acceleration shaping; do not add a second per-sample servo speed
trajectory.

Hardware replay after #74 remained very shaky and A_UP->B_UP still failed during
pre-motion planning at 0.794 mm against the unchanged 0.5 mm tolerance. Review of the
public managed controller confirmed that real Cartesian execution was already using the
intended teleoperation servo profile (`speed_raw=0`, `acceleration_raw=254`), so the
shake remains unresolved and should not be attributed to a servo-profile regression.

Position-only Cartesian planning still filters the solved joint path only to create smoother
IK seeds and re-solves every interior sample at the unchanged hard tolerance. In addition,
if forward sequential IK hits the observed numerical pocket, the planner may solve the same
samples backward from an exact reachable endpoint solution. The paper workflow must pass
the endpoint solution already produced by read-only preflight as that boundary seed. The
reverse path is valid only if it reconnects continuously to the measured start and every
sample passes the same tolerance and safety checks.

The IK solver also performs a task-space-only refinement when soft continuity/joint-center
regularization would otherwise leave a target just outside the hard Cartesian tolerance.
Do not loosen the tolerance to make such a case pass.

Hardware validation of the endpoint-seeded reverse fallback still failed safely:
forward and reverse A_UP->B_UP solves converged to 0.648 mm and 0.646 mm respectively
against the unchanged 0.5 mm tolerance. Before another powered replay, the script must
preflight every elevated segment from its endpoint IK solutions. A failure must identify
the exact command-rate sample and line progress and report signed XYZ residual, positional
Jacobian conditioning, and nearest effective joint-limit margin. The replay-only `--limit-compare-only` diagnostic must also keep torque disabled.
It compares the saved reference-height path under normal effective limits with read-only
calibration-derived extensions inset from the measured mechanical stops. The diagnostic
must never narrow the normal executable range, change executable `move-linear` or
joint-space limits, or accept any explicit read-only bound outside the active calibration.
The current calibration records wider mechanical travel than the nominal model, including
wrist flex ±103.9° versus the model's ±95°. Hardware-side read-only comparison at 107 mm
failed under the old +95° wrist-flex boundary. A completed torque-off margin search found
5.430° feasible and 5.469° infeasible; the largest feasible tested solution used wrist flex
through +98.48° and remained 5.43° from the measured +103.91° stop.

Executable motion now treats URDF/model joint limits as the generic fallback/reference.
When a real arm has a valid mechanical-stop calibration, all five pose joints may use the
measured travel with a 1° inset from each stop; a narrower measured range remains
authoritative. Before powered paper replay, verify `soarm101 limits --json` reports the
expected ~1° inset and that read-only full-segment preflight passes under the normal runtime
limits. This change does not authorize commanding a measured mechanical stop and does not
resolve the separate visible-shake issue.

Also validate the calibrated Sleep posture first in simulation, then with a clear physical
workspace at low speed. After Sleep completes, verify the CLI remains torque-held until the
operator presses ENTER and that ENTER then relaxes the arm. From the held folded posture,
replay a known-safe saved joint pose and verify sample 0 does not block departure solely
because the arm is already inside the coarse self-clearance envelope. The path must fail if
any non-self-clearance workspace guard fails, if minimum self-clearance decreases while
exiting, if the path never reaches the configured clearance threshold, or if ordinary
self-clearance becomes invalid again after the path has cleared it. Sleep is derived from the active executable limits: shoulder pan
midpoint, shoulder lift lower limit, elbow flex upper limit, wrist flex at 75% of its
executable range, and wrist roll midpoint. On a calibrated physical follower the selected
limit-derived endpoints are already 1°
inside the measured mechanical stops. Sleep retains calibrated joint limits plus trajectory, rate/acceleration, following-error,
effort, fault, communication, and completion guards, but intentionally skips the generic
coarse workspace-geometry check. The designed folded posture places link centerlines closer
than the generic 25 mm self-clearance heuristic on this arm, so that heuristic produces a
known false positive for Sleep. This exception is specific to the calibration-derived Sleep
primitive; ordinary joint motion continues to use the coarse workspace check. After the arm
fold completes, Sleep closes the stock gripper to a target 1° inside its calibrated closed
mechanical stop. The 2026-10-06 overload event was later explained by a pen already being
held in the gripper, so it is not evidence that the calibration-derived 1° inset is unsafe.
It is evidence that Sleep is not object-aware: before physical or agent-triggered Sleep,
confirm that closing the gripper is appropriate for the current grasp. Verify
`soarm101 limits --json` reports the derived normalized/raw target. Sleep is never automatic.

The replay-only
`--height-sweep-only` diagnostic must keep torque disabled while it searches for the
nearest constant calibrated workspace Z whose endpoints and all four straight segments
preflight successfully. The saved calibration's 5 mm sweep found the first feasible path at
130 mm (+23 mm above the measured 107 mm reference); candidates through 125 mm remained
wrist-flex limited. Because 130 mm is extrapolated beyond the measured reference, do not
treat the sweep result itself as powered validation. First inspect the printed per-segment
joint step/speed/acceleration/jerk and per-joint encoder quantization/reversal diagnostics.
Only after the straight path itself is shown feasible should visible shake be characterized
with planned joint derivatives, encoder-quantized command deltas, measured following error,
and actual cycle timing before changing motor PID or power settings.

### Paper linear-motion settle criterion

The paper experiment is a supervised qualitative Cartesian-line validation, not a
high-precision metrology test. It therefore uses a local completion tolerance of 3.0°
per joint and an 8 s final settle window. The SDK-wide defaults remain stricter.

The defaults can be overridden explicitly:

```bash
python examples/paper_workspace_calibration.py --replay \
  --settle-tolerance-deg 3 \
  --settle-timeout-s 8
```

Do not increase these further merely to make a failing arm pass. If a move still times
out, use the emitted worst-joint/per-joint errors plus voltage/current/moving/status
diagnostics to distinguish servo authority, power, friction/load, or model problems.

### Paper/workspace calibration — after the basic motion gates

Run:

```bash
python examples/paper_workspace_calibration.py --reference-height-mm 107
```

Teach A->B->C->D clockwise, then teach the fixed lower finger at the measured D_UP point.
The saved workspace calibration levels A_UP/B_UP/C_UP/D_UP/CENTER_UP to that same physical
workspace Z while preserving calibrated workspace X/Y. CENTER_UP must be exactly the
calibrated paper midpoint.

After endpoint preflight and the separately validated startup clearance, the powered
sequence must use `move_linear()` for
`A_UP -> B_UP -> C_UP -> D_UP -> CENTER_UP`. Assert that the requested target workspace
Z is the same for all five targets and record achieved workspace Z after each segment.

Compare physical smoothness directly with 20 Hz teleoperation. Cartesian execution should
use `speed_raw=0` and `acceleration_raw=254` just like teleop while the host path limits
speed and acceleration. There must be no calibrated-hardware branch that silently replaces
those values with proportional per-sample speed caps.

After a teaching run, test:

```bash
python examples/paper_workspace_calibration.py --replay \
  --speed-mm-s 20 \
  --acceleration-mm-s2 100
```

Replay may begin from an ordinary resting pose and must not require reteaching after a
later preflight/motion failure. Use `--measure-only` when a non-moving capture is wanted.

See `docs/workspace-calibration.md` and `docs/validation.md`.


## Batch 1 — Setup, follower control, and leader readout

### 1. Launch and simulation smoke test

1. Pull the latest tested branch/main and install the GUI extra:
   `pip install -e ".[gui]"`
2. Run:
   `soarm101-gui --simulation`
3. Confirm the window opens with Setup, Camera, Manual, Teleoperation, Teach / Record, Edit recordings, Programs, and Log tabs.
4. Connect the follower simulation.
5. Enable it, jog joints/Cartesian axes, move the gripper, and confirm STOP/HOLD and
   Relax still work. Confirm Manual has only **Joint / angular** and **Cartesian** arm
   mode tabs and that the gripper tool panel remains visible in both.
6. Confirm one persistent right-hand follower sidebar is visible while switching through
   Setup, Manual, Teleoperation, Teach / Record, Edit recordings, Programs, and Log. The
   solid arm must always remain the follower. Confirm its five joint values, TCP XYZ/RPY,
   gripper value, state chip, and Enable hold / STOP-HOLD / Relax controls remain visible.
   Shrink the window vertically and verify only the model/readout region scrolls while the
   three safety controls stay pinned. Confirm torque-off follower state reads RELAXED.
7. Confirm active-tab context appears only as a ghost: leader in Teleoperation, saved
   position in Teach / Record, scrubbed recording pose in Edit recordings, and selected
   destination in Programs. Select Leader as the teaching source and verify the solid
   primary model still remains the follower.
8. Open Camera and confirm device, width, height, FPS, FourCC, mirror, auto-start, and
   snapshot-folder controls are present. With no camera connected, the GUI must remain
   responsive and show a stopped/unavailable preview rather than failing startup.
9. Change the gripper speed preset in Manual and confirm the same preset appears in
   Teleoperation, Edit recordings, and Programs.
10. With two named cameras active, confirm each live card reports capture FPS, preview age,
    and preview frames coalesced since the preceding UI refresh. Artificially slow GUI
    presentation or exercise the deterministic mailbox test; after a burst of camera frames,
    the next displayed image must be the newest frame rather than an older queued image.
11. Save the current simulated follower position as Home and Rest.
12. Move away, use Go Home / Go Rest, and confirm the controls target the saved poses.
13. Close and reopen the GUI and confirm Home and Rest are still present.

### 2. Physical follower connection — DO LATER

Before power-on, remove payloads, clear the workspace, secure the base, and keep the
physical power switch/plug immediately accessible.

1. Connect only the follower USB controller.
2. Start `soarm101-gui` and choose the follower serial port.
3. If the arm has never been calibrated, enable the explicit setup/uncalibrated
   connection option. Do not enable torque.
4. Confirm all six motors are detected and the displayed measured positions change
   sensibly when the unpowered arm is moved.

### 3. Mechanical-stop calibration — follower and leader — DO LATER

Follower and leader use the same calibration algorithm and the same Setup panel. Their
calibration files remain separate because each physical arm has its own robot ID and
measured endpoints.

1. Keep both arms torque OFF.
2. Open **Setup > Mechanical-stop calibration** and select **Follower**.
3. In **Setup / Calibrate**, click **Find Arms**, select **Follower**, then click
   **Connect for calibration (torque off)**. This sets the uncalibrated setup option
   for the follower connection. Do not press Enable.
4. Start recording. Six circular gauges reset to 0%. Each gauge fills while the
   encoder moves across its range, resets for the return traversal, and shows
   **DONE** with a brief black pop after two complete end-to-end traversals.
5. Gently move every joint and the gripper from one printed stop to the other and
   back. Do not hold or force a joint against a stop. Allow time for all six
   gauges to reach 2/2.
6. The minimum observed range is 2048 ticks (180°) for each pose joint and
   900 ticks for the gripper. The gripper gauge uses the selected arm's saved
   range as its visual scale when available; the leader and follower need not
   have the same gripper travel. **DONE** means two traversals were observed,
   not that the calibration file has been saved.
7. Recording ends automatically when all six reach 2/2; otherwise the selected
   time limit applies. Wait for **CALIBRATION SAVED** or **CALIBRATION FAILED**.
   Calibration must fail rather than save if any actuator is
   below its current minimum or has fewer than two traversals. A servo input-voltage
   fault also stops the sweep regardless of gauge progress. Otherwise the SDK derives
   each zero from the midpoint of
   the observed extrema, writes symmetric limits, reads them back, and saves calibration
   under the follower robot ID.
8. Record the short calibration ID shown in Setup. Verify
   `~/.config/soarm101/calibration/<follower-id>.json` exists and that an immutable
   fingerprint-named copy exists under
   `~/.config/soarm101/calibration/history/<follower-id>/`.
9. Reconnect the follower normally and confirm its GUI joint sliders use the calibrated
   limits.
10. Select **Leader** in the same Setup panel, then click
    **Connect for calibration (torque off)**. The discovered leader port and its own
    robot ID are used, and torque remains off.
11. In the same Setup calibration panel select **Leader** and repeat the exact sweep
    procedure. The same six gauges and thresholds are used, but the result is saved under
    the leader robot ID.
12. Record the leader calibration ID/history copy, reconnect the leader normally, keep
   its torque OFF, and confirm its readout changes
    sensibly as it is moved by hand.

### 4. First follower motion validation — DO LATER

Do not start with Home/Rest or Cartesian moves.

1. Start near the mechanical midpoint with no payload.
2. Before torque-on, the SDK now reads every selected motor's Present_Position and
   EEPROM Min/Max Position Limits. If any measured position is outside its active
   range, enabling must fail before any goal or torque-enable write. With torque off,
   manually move that joint back inside its calibrated range and investigate the
   calibration if the reported limits are unexpected.
3. Confirm that an intentionally uncalibrated/factory-range setup session refuses torque
   enable. This rejection must occur before any Goal_Position, Lock, or Torque_Enable write.
4. For the calibrated follower, use the CLI smoke test first:
   `soarm101 smoke-test --port PORT --robot-id ROBOT --joint shoulder_pan`.
   Its default motion is only 2° at 0.05 rad/s with 0.20 rad/s² acceleration, then it
   returns and relaxes.
5. Repeat the same 2° CLI smoke test for shoulder_lift, elbow_flex, wrist_flex, and
   wrist_roll one at a time. Verify the **physical positive direction** for each joint
   agrees with the GUI/model convention before any Cartesian testing.
6. After the CLI tests pass, enable in the GUI and make similarly small one-joint moves.
   Press STOP/HOLD during one slow move and confirm the arm stops/holds; Relax must then
   disable torque.
7. Put the relaxed gripper near mid travel by hand. Enable and command small normalized
   changes such as 0.50 → 0.55 → 0.45 → 0.50. Confirm direction and effort behavior.
   Do not use Open/Close endpoints as the first powered gripper test.
8. Only after all five joints and the small gripper test pass should you save fresh
   Home/Rest positions under the current calibration and replay them at low speed.
9. Only after joint-direction, STOP, gripper, Home/Rest, and FK checks pass should you test
   the default 2 mm Cartesian jogs. Start in **World / base** frame with a clear workspace.
10. Test one axis at a time in this order: X−/X+, Y−/Y+, Z−/Z+. After each click, verify
    both the orange target marker in the Manual joint-center view and the requested TCP diagnostic
    move only along the intended world axis before judging the physical arm motion.
11. If the model target is correct but the physical arm moves along the wrong axis, stop
    Cartesian testing and treat it as a joint-direction/calibration-to-model mismatch.
    Capture the GUI session log plus the five one-joint positive-direction observations.
12. After all six directions agree, click the same jog button three times rapidly. Confirm
    the queue reports two waiting commands after the first starts, executes all three
    sequentially, and STOP/HOLD immediately cancels the active jog and clears the remainder.

### 4a. Motor effort/current characterization — DO LATER

Do this after the first small joint-motion checks and before relying on the effort guard
for larger or continuous motion. Raw current/load are safety indicators, not calibrated
force.

1. Open Setup > **Motor effort safety / characterization**. Confirm the guard is enabled
   and note the session defaults and each motor's effective thresholds.
2. With torque OFF and no payload, press **Refresh readings** once. Confirm all six motors
   return plausible raw values and there are no communication errors.
3. Press **Reset peaks**, place the arm near its centered pose, enable torque, and let it
   hold without commanded motion for several seconds. Record the displayed per-motor
   current/load values and peaks.
4. Run the already-validated slow ±5° motion on one joint at a time. After each move,
   inspect the session peaks. The backend samples effort during guarded motion, so the
   peak display should capture values seen by the safety monitor without adding a second
   serial-read loop from the GUI.
5. Keep the default thresholds for the first characterization pass. If normal unloaded
   holding/motion produces false trips, **Relax first**, then make a small explicit
   threshold adjustment and repeat the same test. Do not change effort settings while
   torque is enabled.
6. If a trip occurs, confirm the global status and effort panel show the latched motor and
   reason, and that motion remains blocked. Remove any contact/obstruction before pressing
   **Clear latched trip**. Clearing the latch must not itself move the arm.
7. Test **Reset peaks** between controlled conditions so unloaded hold, single-joint
   motion, gripper motion, and later payload tests can be compared separately.
8. Treat disabling the effort guard as diagnostic-only. The GUI requires explicit
   confirmation, the setting lasts only for the current connection/session, and all
   other SDK safety layers remain active.
9. Record useful provisional per-joint ranges for current and |load|. Only after repeated
   tests should the defaults or per-motor overrides be tightened/relaxed in code.

### 5. Leader readout — DO LATER

1. Connect the already-calibrated follower and leader on separate USB serial adapters.
2. In Setup, select the leader port and its distinct leader robot ID.
3. Connect the leader with torque OFF. If it still needs calibration, use the shared Setup calibration workflow before returning to teaching.
4. Move each leader joint by hand and confirm the displayed angles and gripper position
   update independently of the follower.
5. In Teach / Record, switch the teaching source between Follower and Leader and confirm the current-source readout and solid kinematic arm follow the selected source.
6. Do not enable live teleoperation yet. That is a later implementation/testing stage.

## Implemented after Batch 1

Point teaching, trajectory recording/replay, timeline editing, sequence execution,
guarded live teleoperation, and advanced trajectory editing are now implemented in
software. Their deferred checks are below.

## Batch 2 — Taught points, trajectory recording, and trajectory editing

### Calibration-provenance replay gate — before physical artifact replay

1. Save a fresh follower taught point and confirm its pose JSON contains
   `source_calibration_id` and `target_calibration_id` matching the active follower.
2. Record one leader trajectory only after both arms are calibrated. Its metadata should
   retain the leader source calibration and the follower target calibration.
3. Derived trajectory edits must preserve the source provenance and target binding.
4. Do not reuse old pre-fingerprint physical artifacts. They are expected to fail closed.
5. After any future follower recalibration, existing poses/trajectories/sequences are
   expected to refuse physical replay until explicitly reviewed and re-saved/re-bound.
6. Simulation intentionally ignores this physical provenance gate.


These checks can wait until the Batch 1 hardware checks pass.

For live teleoperation timing, use 20 Hz as the normal hand-following validation rate.
A follower cycle may occasionally exceed the nominal 50 ms period without being unsafe
if queued leader sample age stays low. Treat increasing queued age, stale-sample stops,
communication faults, following-error trips, or delayed STOP/HOLD response as failures.
Use 10 Hz only as a diagnostic fallback; its 100 ms command spacing can be visibly stepped.

### 6. Software-only point teaching

1. Run `soarm101-gui --simulation`.
2. Connect and enable the follower simulation.
3. In Teach / Record, leave Follower selected as the teaching source.
4. Move the simulated follower to a non-home joint pose.
5. Save it as a named taught point such as `test_point_1`.
6. Move away from the point.
7. Replay it in Joint / angular mode and confirm the measured joint state returns to it.
8. Move away again, replay it in Cartesian linear mode, and confirm the TCP returns to
   the saved pose.
9. Restart the GUI and confirm the taught point remains in the pose library.

### 7. Software-only raw trajectory recording and replay

1. Connect the follower simulation and the leader simulation.
2. Select Leader as the teaching source.
3. Enter a unique raw trajectory name such as `sim_wave_raw_01`.
4. Leave effort/current recording off for the first test.
5. Start recording, allow it to run for several seconds, then stop.
6. Confirm the Edit recordings tab opens with a six-lane timeline (five joints + gripper).
7. Confirm the raw file appears under the library as `raw`.
8. Try to reuse the same raw name and confirm the GUI refuses to overwrite it.
9. Enable the follower simulation and replay the raw trajectory.
10. Confirm replay first moves safely to the clip start and then executes the recorded
    stream.

### 8. Software-only trajectory editing

1. Load the raw test trajectory.
2. Scrub through the timeline and verify the cursor time updates.
3. Set selection start/end inside the full recording.
4. Replay only the selection.
5. Set speed scale to 0.5x and replay the selection; duration should approximately double.
6. Save the selection as a new edited name such as `sim_wave_trim_v1`.
7. Confirm both the raw and edited versions remain in the library.
8. Reload the raw version and confirm it was not changed by the edit.
9. Load the edited version and replay it.
10. Try a faster speed scale. If the resulting trajectory exceeds configured joint
    speed, acceleration, or command-step limits, replay should be rejected rather than
    silently retimed.

### 9. Physical taught-point replay — DO LATER

Only continue after Batch 1 first-motion validation passes.

1. Teach one point from the follower while torque is off, then enable torque.
2. Move a small distance away and replay the point in Joint mode at low speed.
3. Verify physical direction and endpoint before trying a larger move.
4. Teach a second point with at least 20 mm of clear workspace around the whole path.
5. Replay it in Cartesian linear mode at low speed.
6. Keep a hand at the physical power switch throughout the first tests.

### 10. Physical leader recording — DO LATER

1. Keep leader torque OFF and follower disconnected or relaxed for the first capture.
2. Record a slow 3–5 second leader motion at 50 Hz with effort/current recording OFF.
3. Confirm the reported median sample rate is close to 50 Hz and that no capture errors
   are logged.
4. Inspect/crop the recording before any follower replay.
5. Put the follower near the recorded start area, enable it, set playback speed to 0.5x,
   and replay with the workspace clear.
6. Confirm the pre-roll is smooth and the replay has no discontinuous jump into sample 1.
7. Press STOP/HOLD during a slow replay and confirm cancellation/hold works.
8. Only after basic recording is stable should you enable effort/current recording.
   Compare achieved sample rate and logs; disable the diagnostic channel if serial load
   makes 50 Hz capture unreliable.

### 11. Physical edited-motion replay — DO LATER

1. Crop a known-safe physical recording to a shorter region and save it under a new name.
2. Replay at 0.5x first.
3. Verify the crop boundary does not create a joint jump; the SDK should reject it if it
   violates command-step/speed/acceleration limits.
4. Test 1.0x only after the slower replay passes.
5. Treat >1.0x playback as a separate validation: faster timing is allowed only when all
   configured hard limits still pass.

## Batch 3 — Programs, live teleoperation, and advanced motion primitives

### 12. Software-only saved-position Program editor and runner

1. Run `soarm101-gui --simulation`, connect the follower simulation, and enable it.
2. Ensure Home/Rest and at least two saved positions exist, for example `above_pick`
   and `pick`.
3. Open Programs. In **Position steps**, build a short linear Program such as:
   Move above_pick → Move pick at 0.5× → Close gripper → Wait 0.25 s →
   Move above_pick → Open gripper.
4. Confirm the list reads as explicit MOVE / GRIPPER / WAIT rows and the selected Move
   destination appears as a ghost in the Program kinematic card.
5. Save the Program, clear/load it again, and confirm step order, movement mode, and
   per-Move speed are preserved. Internally this remains a `MotionSequence`.
6. Use **Run selected step** on each step individually.
7. Run the entire Program once at 0.5× overall speed, then at 1.0×.
8. Set Repeat to 2 and confirm the complete Program runs twice.
9. Add a WAIT of at least 1 second. Start the Program, press
   **Pause after current step**, and confirm it pauses at a step boundary or pauses the
   WAIT timer. Resume and confirm execution continues.
10. During another run press **STOP / HOLD** and confirm the active Program is cancelled
    and the follower holds.
11. Restart the GUI and confirm saved Programs remain available.
12. Select a saved base position in Programs and generate a radial shoulder-pan
    pattern from -30° to +30° in 15° increments. Confirm each generated row references
    the same base position, changes only shoulder pan, previews the overridden pose as a
    ghost, and is rejected if a generated shoulder-pan target exceeds the current limits.
13. Run the radial rows one at a time in simulation and confirm shoulder lift, elbow,
    wrist flex, and wrist roll remain equal to the saved base position.
14. If a saved trajectory exists, open **Trajectories / primitives**, add it as one
    Program step, and confirm it executes through the same guarded runner. Confirm
    recording, replay, editing, and trajectory-library persistence remain available
    independently of the position Program workflow.

### 13. Software-only advanced trajectory editing and primitives

1. Load a known trajectory in Edit recordings.
2. Apply a 5-sample smoothing window. Confirm the displayed shape changes while the
   first and last poses remain unchanged.
3. Select an interior region and use Delete. Save As a new edited trajectory.
4. Reload the source trajectory to confirm it was not modified.
5. Insert a 0.5 second hold at the scrub cursor and confirm duration increases by about
   0.5 seconds.
6. Set a small joint keyframe at the cursor, or set a gripper keyframe within [0, 1].
   Replay validation remains authoritative; deliberately extreme edits should be rejected.
7. Add markers such as `contact`, `beat`, or `release` and confirm they appear on
   the timeline.
8. Select a region and make a 2× repeated clip. Inspect the loop boundary before replay;
   a discontinuous loop must be rejected by normal motion safety validation.
9. Save a safe edited trajectory, give it a semantic name such as `wave_gentle`, add
   tags, and promote it to a motion primitive.
10. In Programs > **Recorded motion · advanced**, add that primitive as a Program step and execute it in simulation.

### 14. Software-only live teleoperation lifecycle

The automated tests exercise moving streaming targets directly. The GUI simulation is
mainly a lifecycle/UI check because the simulated leader has no physical hand input.

1. Connect follower simulation and leader simulation.
2. Leave the follower connected with torque off; starting teleoperation should latch its measured pose, enable hold, and run the alignment step.
3. In Teleoperation choose **Relative / clutch-safe** and leave Mirror gripper enabled.
4. Confirm **20 Hz — default** and **Medium · current** Tracking response are selected, then click **Align follower and start**. Confirm the status reports alignment before live following and ordinary follower jog/sequence controls are disabled while teleop is active.
5. Stop live teleoperation and confirm the follower returns to holding state.
6. Start it again and press the global **STOP / HOLD**. Confirm teleop ends.
7. Disconnect the leader while teleop is active. Confirm follower teleop terminates and
   holds rather than continuing with stale targets.
8. Repeat the lifecycle with **Absolute calibrated angles** as a software check; on hardware,
   Absolute mode is deferred until calibration/alignment validation below.
9. Stop teleoperation. Click **Park leader here** and confirm leader simulation reports
   PARKED / torque on; click **Release leader** and confirm FREE / torque off.
10. Put the simulated leader and follower at different poses. With gripper matching
    unchecked, use **Move leader → follower pose** and then the reverse direction; confirm
    each destination converges while the source remains unchanged.
11. Diverge the two simulations again and use **Relink here — no motion**. Confirm neither
    arm performs an alignment move before relative teleoperation begins.
12. While teleoperation is active, click **Stop → Manual + park leader**. Confirm the
    follower remains holding, Manual opens, and the leader reports PARKED.
13. Exercise Slow, Normal, and Fast gripper presets through a manual move, a Program
    gripper step, and recorded-trajectory replay; inspect simulator/backend tests for the
    raw speed propagation because simulation itself has no motor-speed dynamics.
14. Exercise **Slow · gentle**, **Medium · current**, and **Fast · wrist-aware** Tracking
    response in simulation and confirm the stream rate does not change when only the response
    preset changes. Medium must preserve the historical 1.2 rad/s / 6.0 rad/s² limiter.
    Fast must expose 100 deg/s to every joint, 1000 deg/s² to the four non-wrist-flex joints,
    and 500 deg/s² to wrist_flex in both the GUI limiter and MotionController stream state.
15. Exercise the stream reversal regression: strict opposite-direction motion still fails,
    but a recent genuine command reversal may accept at most 100 ms of non-growing physical
    carry-through, never more than 0.10 rad cumulative wrong-way travel, before the ordinary
    direction fault resumes.

### 15. Physical Program / radial-pattern execution — DO LATER

Before a complete physical Program, validate every saved destination individually at
conservative speed. For a radial pattern, first test the smallest shoulder-pan offset,
confirm all four non-pan arm joints remain at the base pose, then expand the range one
step at a time. The generator checks configured/calibrated pan limits but does not know
about external fixtures, cables, payload collisions, or table geometry.


Only continue after the Batch 1 and Batch 2 physical checks pass.

1. Build a sequence containing only Home, one already-validated small taught point, a
   short WAIT, and Home.
2. Set global sequence speed to 0.5× and Repeat to 1.
3. Keep the physical power switch accessible and execute each step with
   **Run selected step** before running the whole sequence.
4. Run the full sequence once.
5. Test **Pause after current step** and Resume.
6. Test **STOP / HOLD** during a slow joint step and during a WAIT.
7. Add a previously hardware-validated trajectory only after the point sequence passes.
8. Increase speed/repeat only after single-run behavior is repeatable.

### 16. Physical relative leader → follower teleoperation — DO LATER

This path now has partial physical validation on the user's follower. On 2026-10-06,
20 Hz Medium tracking completed cleanly and felt substantially better than the earlier
conservative response. A subsequent 20 Hz Fast run improved general tracking but exposed a
wrist-flex-specific reversal limit: with the former 1000 deg/s² Fast wrist response, the
controller observed +0.057 rad of wrist motion in the previous physical direction after a
commanded reversal and correctly stopped. The next hardware gate uses Fast · wrist-aware
(100 deg/s, 500 deg/s² on wrist_flex) plus the bounded stream-only reversal braking grace.
Run it with no payload and gripper mirroring disabled first.

1. Complete calibration, joint-direction, FK, low-speed joint, and STOP checks first.
2. Secure both bases, clear the follower workspace, remove payloads, and keep physical
   follower power immediately accessible.
3. Connect both arms. Keep the leader torque OFF and leave the follower torque OFF before starting teleoperation.
4. Before live teleoperation, validate parking by holding the leader near a safe supported
   pose, clicking **Park leader here**, and confirming it does not jump when torque enables.
   Click **Release leader** and confirm it becomes back-drivable again. Do not continue if
   parking causes unexpected motion.
5. With both arms already close to one another and the gripper-match box unchecked, test
   **Move leader → follower pose** at the conservative default synchronization speed, then
   test the reverse direction. Keep physical power accessible and validate each destination
   joint direction before increasing pose differences.
6. Deliberately leave the arms at slightly different safe poses and test **Relink here —
   no motion** in Relative mode. Confirm the follower only latches its own measured pose;
   there must be no alignment move before leader motion begins.
7. Put both arms in comfortable poses with the leader inside the follower's calibrated travel. Keep the leader still during startup and keep the follower in a clear nearby pose so the guarded alignment move is small.
8. Select **Relative / clutch-safe**, initially disable gripper mirroring, select
   **5 Hz — slow check**, and click **Align follower and start**. Confirm torque enable does not cause a jump and the guarded alignment completes (or reports a staged offset near a model limit) before live following begins.
9. Move only one leader joint a few degrees, slowly. Confirm the follower moves the same
   signed delta after alignment.
10. Watch the live follower-cycle and queued-age values. At 5 Hz the period is 200 ms;
    processing should remain comfortably below that and queued age should stay low rather
    than increasing over time.
11. Return that joint and repeat for the other four joints one at a time.
12. Test STOP/HOLD while making a slow motion. The follower must stop/hold and teleop must
    terminate.
13. Restart teleop, then disconnect/unplug the leader data connection. The follower must
    stop receiving stream targets and hold.
14. Re-enable gripper mirroring and test a small leader gripper delta.
15. After 5 Hz is repeatable, run the same checks at 10 Hz (100 ms period).
16. At 20 Hz, verify **Medium · current** still reproduces the previously successful
    behavior, then select **Fast · wrist-aware** with gripper mirroring disabled. Sweep
    wrist_flex alone through several reversals and inspect `teleop_frame` leader, desired,
    command, command velocity, actual, and following-error fields. Confirm wrist command
    acceleration does not exceed 500 deg/s² and that any opposite-direction carry-through
    after reversal is brief/non-growing rather than a persistent drift.
17. Repeat coordinated five-joint Fast motion only after the isolated wrist test passes.
    Then re-enable gripper mirroring at Normal gripper speed; keep object contact tests
    separate from wrist-response testing.
18. Treat 50 Hz (20 ms period) as a separate experimental stage. Do not increase merely
    because motion looks smooth; record cycle time, queued sample age, overruns,
    communication errors, STOP response, following errors, and effort trips as described
    in `docs/teleoperation.md`.
19. If follower processing exceeds the selected period repeatedly or queued sample age
    grows, the software should terminate teleop and hold. Reduce the rate before retrying.
20. Deliberately move the leader faster only enough to verify configured step/speed/
    acceleration guards reject unsafe streaming rather than following it.
21. Do not treat software STOP as an emergency stop; physical power remains the ultimate
    intervention during these tests.

### 17. Physical absolute teleoperation — DO LATER, AFTER RELATIVE PASSES

1. Verify leader and follower use compatible calibrated joint signs and zero conventions.
2. Place the follower in a clear pose reasonably near the leader to keep automatic alignment travel small, and keep the leader still during startup.
3. Select **Absolute calibrated angles** at 5 Hz and click **Align follower and start**. The start sequence should latch the follower before torque enable and perform a guarded alignment; if the leader pose is outside the follower's calibrated travel it must refuse before live following. If the initial difference exceeds command-step or
   other stream limits, rejection is expected and preferable to a jump.
4. Test only a few degrees on one joint at a time before coordinated motion.
5. Repeat the STOP and leader-readout-loss tests from Relative mode.

### 18. Physical advanced motion primitives — DO LATER

1. Start from a trajectory already proven safe on hardware.
2. Apply one edit at a time and save every version under a new name.
3. Replay smoothing/hold/keyframe/splice variants first at 0.5×.
4. Inspect repeated/looped clips especially carefully at the end→start boundary.
5. Promote only hardware-validated edited trajectories to show motion primitives.
6. Run a primitive first by itself, then as a single sequence step, then in a multi-step
   sequence.
7. Record which primitive versions, speed ranges, and payload conditions have actually
   passed hardware testing.

## Software-complete / hardware-validation pending

The teaching workflow is now implemented through the Run/primitive layer. Remaining work
is physical validation and future optional capabilities such as Cartesian velocity
streaming, richer mesh collision models, and show-level orchestration in the appropriate
Robo Puppeteer/Director repositories.


## Live GUI motion visualization

Automated coverage verifies that MotionController and the stock gripper publish motion-owned
feedback during guarded execution, and that partial GUI live updates animate the arm joints,
model-estimated TCP readout, and jaw aperture while preserving the latest full-state snapshot.
Recorded/alignment paths may add a gripper sample at an existing controller feedback checkpoint,
but must never start an independent hardware polling loop.

Local GUI acceptance:
1. connect the follower and leave the sidebar visible;
2. command a small joint move and confirm the arm schematic and TCP readout move during transit;
3. command gripper open/close and confirm both the jaw schematic and numeric aperture move
   continuously rather than only jumping after completion;
4. run 20 Hz teleoperation and confirm measured joints and mirrored gripper remain live;
5. confirm no new communication errors or timing misses appear from visualization activity.

The live-display path must not add a second serial poller while a motion handle owns feedback.

## Guided Setup GUI validation

Automated checks:

```bash
pytest tests/test_gui_setup_panel.py tests/test_setup_backup.py tests/test_gui_calibration_flow.py tests/test_gui_sessions.py --no-cov
```
Regression coverage includes missing/corrupt/valid calibration, no implicit connection
on navigation, optional guidance, palette contrast, offline model labeling, backup
fingerprint/path rejection, previous-file preservation, and rollback on write failure.

Local workstation acceptance: run `soarm101-gui` at normal scale and enlarged desktop
text, in both light and dark themes. At 1080×720 and full screen, verify Setup is readable,
sidebar STOP/HOLD stays visible, and calibration gauges reflow. With existing arms,
verify discovery assignments, saved-vs-loaded calibration status, explicit connection,
and calibration reuse. Open/leave calibration without starting a sweep; verify no
motion. With a supervised calibration session, check cancellation and completion, then
normal reconnection. Export a backup to a separate location; inspect/restore a copy only
with arms disconnected. Verify restart and matching calibration provenance. Do not
recalibrate a working arm solely for a cosmetic smoke test.

## GUI control contrast — visual validation

1. Launch Motion Studio with the normal desktop theme.
2. Check every tab with both enabled and disabled controls visible.
3. Confirm ordinary buttons have a distinct filled surface and border rather than appearing
   as label text.
4. Confirm primary task actions such as Connect, Find Arms, Align/Start, Save, Run, and
   Cartesian Move use the accent-filled primary treatment when enabled.
5. Confirm STOP / HOLD uses the danger treatment when enabled.
6. Confirm disabled controls remain visibly button-shaped while reading as unavailable.
7. Hover/focus several controls and confirm the state change is obvious without changing
   layout size.

## Persistent operating-status banner

The top status banner must remain visible in normal operation, including when no error is
active. Verify the neutral/green/amber base text follows follower connection, holding/moving,
and teleoperation linked/delinked state. Trigger a synthetic GUI error and confirm **Clear**
appears; pressing it must remove only the transient notice and leave the banner visible.
For a teleoperation stale-sample stop, clearing the red notice must fall back to the amber
TELEOP STOPPED / follower holding / relink-required state. For an active robot fault, clearing
any prior notice must still show the non-dismissible fault state. The complete event remains
in Log regardless of banner acknowledgement.

## Teleoperation workspace layout and live diagnostics

Software/offscreen checks should confirm the persistent right sidebar is 430–520 px wide,
the follower model has a larger minimum viewport, the coordination and teleoperation-setting
groups share the top row, and two configured named cameras occupy the first row side by side.
Teleoperation must provide Start all / Stop all camera controls without creating duplicate
camera workers. Feed synthetic leader stream samples and follower joint measurements and
confirm the 10-second Motion trace accepts samples in both Joint angles and Tracking error
modes. The trace must not initiate motor/register reads.

Local GUI review with two configured cameras: open Teleoperation at the normal desktop size,
confirm both camera cards are visible simultaneously, Start all starts both existing sessions,
and the lower trace remains readable while the right robot model is visibly larger. Current/load
diagnostics remain in the existing effort/recording workflows rather than being added as a
new high-rate GUI poller.

## GUI model geometry and scale

```bash
QT_QPA_PLATFORM=offscreen python -m pytest tests/test_model_geometry.py tests/test_gui_model_geometry.py tests/test_gui_sessions.py -o addopts=""
```

The tests compare native FK with the bundled URDF and verify fixed-finger/rotating-jaw
behavior, stable scale under pose/ghost/target changes, standard views, and active TCP.

Local GUI review: open simulation; compare Side/Front/Top/Isometric, drag, zoom,
double-click reset, Fit, and Auto fit at normal and smaller window sizes. With Auto
fit off, pose and target changes must not alter ruler length. Confirm link ends are flat and
no longer form oversized round blobs where visual elements overlap; the smaller joint
markers should remain clearly distinguishable from the link bodies. Confirm the shoulder,
elbow, wrist-flex, and wrist-roll pivots appear within the corresponding compact motor-case
envelopes rather than at forced bar endpoints. The upper and lower printed members may be
visibly offset from those pivots; that is intentional and should remain stable across
Side/Front/Top views. Jaw opening must leave
the fixed finger still and rotate only the moving jaw; in the normal light theme the moving
jaw should read as neutral dark/black and slightly heavier than the fixed finger. Confirm the
model Z=0 label and readable toolbar/ruler. No hardware motion is needed for these presentation checks.
Physical TCP/geometry and shaking validation remain separate tasks in PLAN.md.

## Camera stream rate and lifecycle — workstation / hardware

1. Start one named camera at its requested format and confirm the negotiated format and
   measured capture FPS shown by the GUI.
2. Confirm preview age stays bounded and note the superseded-frame count while the camera tab
   is visible.
3. Stop and restart the stream through the GUI several times while monitoring kernel USB/UVC
   events; record any disconnects, URB resubmission errors, `-71` errors, or re-enumerations.
4. When validating camera behavior alongside robot control, first observe both arms with
   torque disabled and serial state polling active, then exercise only normal guarded motion
   with no payload and a clear workspace.

## USB camera disconnect / reconnect recovery — workstation / hardware

1. Save the camera using its stable `/dev/v4l/by-id/...-video-index0` identifier and start the
   live stream.
2. Physically unplug that camera while it is streaming. Confirm its preview card changes to
   **WAITING**, the last good image remains visible, and the GUI stays responsive.
3. Reconnect the same camera within 30 seconds. Confirm the same logical camera profile
   automatically reopens and returns to LIVE without pressing Start again.
4. Repeat with both named cameras running and confirm the unaffected camera continues streaming.
5. Leave the camera disconnected beyond the 30-second recovery window and confirm the stream
   becomes STOPPED with a visible terminal error instead of retrying forever.
6. If practical, reproduce a V4L2 reopen failure such as `VIDIOC_REQBUFS ... ENODEV` and
   confirm it uses the same bounded reconnect window after the camera had previously been live.

## Camera transient-drop recovery — workstation / hardware

1. Start one camera and confirm normal live preview.
2. While streaming, induce a brief USB/frame hiccup if practical (for example, momentary host
   load or a known camera that occasionally returns an empty frame). A single missed frame
   must not stop the stream.
3. Confirm a transient miss shows RECOVERING briefly and the previous preview remains visible.
4. Request a snapshot during/around a transient miss and confirm the snapshot completes on the
   next good frame rather than disappearing.
5. If the device produces repeated consecutive empty frames, confirm the worker attempts to
   reopen it while keeping the stream in RECOVERING state.
6. Confirm a camera that repeatedly opens but never returns usable frames stops after the
   bounded recovery cycles with a visible terminal error rather than retrying forever.
7. Confirm a genuinely unavailable/unopenable camera also stops after its bounded open retries.

## Camera layout / multi-view validation — workstation / hardware

1. Open Camera with one saved camera and confirm the setup controls stay in a compact panel
   (roughly 820 px maximum) rather than stretching across the whole workspace.
2. Confirm **USB device** is visibly a dropdown and is populated on tab construction; press
   **Find cameras** and verify the same stable devices remain available.
3. Configure two named cameras such as `overhead` and `wrist`, start both, and confirm the
   lower preview area splits into compact side-by-side cards with name/status visible for each.
4. Add a third saved camera profile (hardware stream optional) and confirm the preview switches
   to a two-column grid without losing the first two cards.
5. Rename/delete a camera and confirm the grid follows the saved registry without stale cards.
6. Switch the selected profile while streams run and confirm selection changes controls/status
   without hiding the other live camera cards.

## Workstation + multi-camera validation — workstation / hardware

Camera capture is independent of powered robot motion, but USB/UVC enumeration, concurrent
bandwidth, and the visual layout require local workstation validation.

1. Pull this version and run `soarm101 workstation show --json`. Confirm an existing legacy
   single-camera setting appears as the migrated `camera` profile when appropriate.
2. Open the GUI. In Setup, connect the follower and leader successfully, then close/reopen the
   GUI and confirm their ports and robot/calibration IDs are restored from
   `~/.config/soarm101/workstation.json`.
3. Open Camera and press **Find cameras**. On Linux, prefer the stable
   `/dev/v4l/by-id/*-video-index0` entries when shown.
4. Rename/save one physical camera as `overhead`. Create a second profile named `wrist`
   and assign a different physical device. Confirm duplicate device assignment is rejected.
5. Configure each camera's resolution/FPS/FourCC/mirroring and snapshot folder. Save both,
   close/reopen the GUI, and confirm all settings and names persist.
6. Start `overhead` and `wrist` individually. Then use **Start all** and confirm both remain
   live concurrently without repeated device-open errors. If USB bandwidth limits the selected
   modes, lower format/rate rather than treating driver failure as a motion-SDK issue.
7. In Camera, switch the selected profile while both streams are live and confirm the preview
   changes to the selected named stream.
8. In Teleoperation, switch the camera selector between `overhead` and `wrist` and confirm
   it reuses the already-running sessions rather than opening duplicate devices.
9. Capture stills from each named camera and verify the files are fresh and correctly oriented.
10. Stop the GUI camera sessions or close the GUI, then run:

```bash
soarm101 camera list --json
soarm101 camera show --json
soarm101 camera capture --name overhead --json
soarm101 camera capture --name wrist --json
soarm101 camera capture --all --json
```

11. Confirm CLI JSON reports logical camera names as well as physical devices and that
    `capture --all` returns one capture record per configured camera.
12. Review Setup/Camera/Teleoperation/Teach/Manual/Programs visually: push buttons should be
    clearly distinguishable from labels/fields, while longer explanatory material should be in
    tooltips or `?` help controls rather than permanently occupying the main layout.
13. Only after these checks should an external agent client be given named camera + motion CLI
    access.

## Torque-enable endpoint tolerance — hardware

1. With torque OFF, place a calibrated joint at or very near one mechanical endpoint.
2. If the reported Present_Position settles 1–8 encoder ticks just beyond the active
   EEPROM Min/Max limit, torque enable should succeed by latching the nearest in-range
   endpoint rather than requiring manual repositioning.
3. Confirm the arm does not make a meaningful jump; the correction is at most about 0.7°.
4. A position 9 or more ticks beyond the EEPROM limit must still fail before any goal or
   torque-enable write.
5. This tolerance applies only to the measured startup latch. Normal commanded positions
   remain constrained to the calibrated limits.

## Torque-enable transient status-packet validation — hardware

1. Connect follower with torque off and verify diagnostics are otherwise clean.
2. Start teleoperation several times from a relaxed follower.
3. A one-off Feetech status-packet error during `Torque_Enable` or `Lock` should be recovered
   only when readback confirms the requested bit or one bounded retry succeeds.
4. Confirm persistent communication errors still abort startup, leave teleoperation inactive,
   and roll back torque on every motor that may have been enabled.
5. Confirm no automatic retry is applied to motion-position writes or calibration EEPROM writes.

## Direct-agent CLI validation — software only

Before giving a coding agent physical serial access, verify the structured command surface
without hardware:

```bash
soarm101 read --simulation --json
soarm101 ik --simulation --x-mm 180 --y-mm 0 --z-mm 160 \
  --orientation-mode position_only --json
soarm101 jog --simulation --x-mm 2 --json --yes
soarm101 gripper --simulation 0.5 --json --yes
```

Confirm each command emits valid JSON and that `ik` performs no motion. Physical one-off
CLI testing remains subject to the earlier joint-direction, FK/TCP, and Cartesian gates;
the GUI and CLI must not own the follower serial port simultaneously.

### Command-rate Cartesian IK smoothness

After hardware comparison showed that the paper traversal remained visibly shaky even
with 1 mm Cartesian IK knot spacing, `move_linear()` was changed to solve IK at the final
host command rate. The latest hardware run remained shaky even though the public managed
controller already used fixed `speed_raw=0`, `acceleration_raw=254`. Regression coverage
therefore continues to require every emitted command sample to be a direct IK solution of
its corresponding Cartesian sample, while position-only planning now also has an
endpoint-seeded reverse fallback for a forward numerical IK pocket without relaxing the
hard tolerance. Physical smoothness still requires real-arm validation.


### Paper replay from a low/resting start

The startup clearance move must be evaluated in the saved calibrated workspace, not by
assuming raw model Z is physical height. Replay requests one 20 mm calibrated-Z rise by
default from the measured starting pose. Regression coverage also includes a case where
the target remains at negative model-frame Z while calibrated workspace Z rises.
Preflight accepts the clearance only when workspace X/Y stays fixed, physical Z does not
descend, IK remains continuous, and all solved joints remain inside effective limits.
After execution, the measured workspace Z rise itself is the acceptance criterion. With
the default 20 mm command, replay requires at least 10 mm measured rise before paper travel
begins at A_UP. Hardware evidence for this threshold is explicit: the dragging case rose
only about 5.4 mm, while the later visually acceptable startup rose about 14.0 mm. The
generic coarse workspace check is disabled only for that verified startup-lift execution.
For elevated paper traversal, the calibrated workspace defines the constant-height
Cartesian targets and the SDK uses `move_linear(..., workspace_check="target_only")` for
the generic model envelope. This retains a generic destination check without allowing the
known-invalid model table floor to veto the measured paper frame. The full joint/IK/
dynamic/following-error/effort/fault/communication/timing safety stack remains active.


### 100/1000 envelope and responsive fallback regression

The host envelope is 100 deg/s / 1000 deg/s² for joints, 100 mm/s / 1000 mm/s²
for TCP translation, and 100 deg/s / 1000 deg/s² for tool orientation. The arm
backend fallback itself is now 0/254, matching streamed/teleop tracking, so direct
guarded arm writes cannot regress to the historical 250/20 throttle. The stock
gripper independently preserves 250/20 default pacing.

Automated coverage must verify the backend fallback, explicit streamed 0/254 behavior,
human-unit CLI envelope reporting/overrides, broker propagation, and gripper independence.
Physical validation now has a successful 80 deg/s / 500 deg/s² Sleep-vs-sleep_up run; the remaining full-envelope gate is the supervised 100 deg/s / 1000 deg/s² Sleep-vs-sleep_up
rerun on the exact branch head.

## Broker watchdog and experimental physical agent mode (software evidence)

Run without hardware:

```bash
python -m pytest --no-cov -q tests/test_broker_watchdog.py tests/test_broker_sdk.py \
  tests/test_broker_stop_faults.py tests/test_broker_session_lifecycle.py \
  tests/test_agent_sandbox.py tests/test_cli.py
```

Coverage includes monotonic expiry, late heartbeat refusal, authority expiry,
replacement/revocation/corruption, terminal SDK faults, independent in-flight STOP,
queued and late success suppression, delayed facade/controller preflight admission,
failed HOLD, failed lease-file deletion,
one-shot STOP evidence, token separation, private endpoint rejection and physical
profile/rate boundaries. A real broker subprocess with simulated motors verifies
that stopping the supervisor pulse triggers broker STOP without another request.
Hermes/Codex × robotctl/MCP runner tests use fake OpenShell, not paid model calls.
Fake-arm physical-route tests verify persistent ownership, reuse of human HOLD
and refusal of torque-off state without enabling torque. These tests never connect
a serial device or prove physical braking. See `docs/physical-run.md` for the
separate attended qualification gate.

Local OpenShell transport check (2026-10-22): real temporary Codex and Hermes
sandboxes successfully imported the client-only MCP package, discovered exactly
health/capabilities/state, and read a persistent simulated SDK session through the
scoped relay. Codex CLI reported 0.162.1; Hermes reported v0.21.4. Both temporary
sandboxes were deleted. The existing local Hermes image initially lacked
`/opt/soarm101-mcp/bin/python`; rebuilding the image from the current packaged
Dockerfile fixed the check. No reasoning-model task, physical connection, camera
capture or physical authority was used in this transport check.

CLI `--simulation` uses the deterministic non-real-time backend so ordinary
software checks do not depend on host scheduling latency. Explicit real-time
simulation (`sim-demo --realtime`) continues to enforce the unchanged 50 ms
command lateness guard; do not relax motion guards to accommodate a busy host.

Final sequential validation (2026-10-22): **797 tests passed** on Python 3.10.20
and **797 tests passed** on Python 3.12.13, both with **82.01%** coverage (70%
required). Ruff, compile checks and source/wheel builds passed. The wheel includes
the watchdog module and both packaged agent Dockerfiles. An additional mypy
comparison found no new diagnostics in the changed modules; existing repository
type errors remain outside this change.

Read-only asset catalog software tests (no hardware):

```bash
pytest -o addopts='' tests/test_asset_catalog.py tests/test_calibration_provenance.py tests/test_trajectories.py tests/test_poses.py
```

These tests cover descriptor ordering, calibration status, content mutation, and
snapshot races. They do not qualify broker replay or physical motion.

Broker asset-catalog software checks:

```bash
pytest -o addopts='' tests/test_broker_asset_catalog.py tests/test_capability_profile.py tests/test_broker.py tests/test_mcp_server.py tests/test_asset_catalog.py
```

These use fake libraries and an in-process MCP client; they do not exercise
physical motion or establish replay authority.

Replay policy admission primitives, without robot motion:

```bash
pytest -o addopts='' tests/test_replay_policy.py tests/test_asset_catalog.py tests/test_broker_asset_catalog.py
```

These do not exercise replay or qualify physical limits.

Offline replay sequence preflight checks:

```bash
pytest -o addopts='' tests/test_replay_preflight.py tests/test_replay_policy.py tests/test_asset_catalog.py tests/test_broker_asset_catalog.py
```

They cover pinned dependencies, calibration, step and duration bounds, entry
metadata, unsupported steps and mutation during preflight; they do not execute
sequence playback or prove physical safety. The preflight report also includes
an ordered step manifest with the revision of each pose or recording dependency,
so future execution can preserve interleaving without resolving mutable names.

Recording preflight tests include stale revisions, calibration mismatch, entry
joint/gripper mismatch and known recording duration. They use supplied fake state,
not measured hardware feedback.

Pinned recording snapshot and preflight tests:

```bash
pytest -o addopts='' tests/test_replay_snapshot.py tests/test_replay_preflight.py tests/test_replay_policy.py tests/test_asset_catalog.py tests/test_broker_asset_catalog.py
```

Simulation-only recording helper and watchdog tests:

```bash
pytest -o addopts='' tests/test_simulated_replay.py tests/test_replay_snapshot.py tests/test_replay_preflight.py tests/test_replay_policy.py tests/test_broker_watchdog.py
```

They include simulated completion, STOP during preflight, deadline and lease
revocation races with deliberately delayed completion. They do not test an agent
broker replay route, all fault cleanup, or physical hardware.

Simulated replay failure cleanup and existing dual arm/gripper STOP tests:

```bash
pytest -o addopts='' tests/test_simulated_replay.py tests/test_broker_stop_faults.py tests/test_broker_watchdog.py
```


Broker recording replay software acceptance (no hardware):

```bash
.venv/bin/pytest --no-cov -q tests/test_broker_replay.py tests/test_simulated_replay.py tests/test_replay_snapshot.py tests/test_replay_preflight.py tests/test_replay_policy.py tests/test_broker_watchdog.py tests/test_broker_stop_faults.py tests/test_trajectories.py
PATH="$PWD/.venv/bin:$PATH" .venv/bin/pytest -m 'not hardware' -q
.venv/bin/ruff check .
```

The broker tests cover explicit grants, strict payloads, pinned revisions and
mutation, same-session reuse, rate forwarding, request retention/conflicts,
lease and heartbeat requirements, queued STOP, lease/deadline/STOP late-success
races, failure cleanup, relay denial and in-process MCP forwarding/filtering.
These are software evidence only; packaged stdio and clean-install replay acceptance,
durable restart reconciliation and physical qualification remain outstanding.
Ordered sequence execution is now covered by the broker tests added here, in
simulation only. Existing helper-only test descriptions above refer to those
earlier isolated tests, not to the broker tests added here.

M2 sequence execution addition (simulation/fake only):

```bash
.venv/bin/pytest --no-cov -q tests/test_broker_replay.py tests/test_replay_preflight.py tests/test_simulated_replay.py
.venv/bin/ruff check src/soarm101_motion/broker.py src/soarm101_motion/capability_profile.py src/soarm101_motion/mcp_server.py src/soarm101_motion/robotctl.py src/soarm101_motion/simulated_sequence.py tests/test_broker_replay.py
```

The added sequence cases cover a saved pose, gripper, wait and recording in
manifest order, request-ID reconciliation, and an independent STOP during pose
execution with late success rejected. These remain software-only evidence.

M2 broker replay verification (2026-10-29, Python 3.14.4, NumPy 2.5.3,
MCP 2.3.0; dependency pins unchanged):

- `PATH="$PWD/.venv/bin:$PATH" .venv/bin/pytest -m 'not hardware' -o addopts='--cov=soarm101_motion --cov-report=term --cov-fail-under=70' -q`:
  **859 passed**, **82.60% coverage**, including all 99 focused replay,
  preflight, policy, snapshot, watchdog, STOP and trajectory cases.
- `.venv/bin/ruff check .`, `git diff --check` and compilation of changed
  implementation modules passed.
- Targeted mypy could not check project code: installed NumPy stubs use a
  Python 3.12 `type` statement while the repository targets Python 3.10
  (`numpy/__init__.pyi:737`). This environment compatibility gap remains.
- An initial full run lacked `.venv/bin` on PATH, causing three existing
  `agent-as-code --check` subprocess tests to miss `soarm101`. Correcting PATH
  resolved them. A symlink diagnostic compatibility regression introduced during
  implementation was fixed before the successful final run.
- No hardware, live robot service, physical arming, commits, push or PR operations occurred.

M2 durable simulated replay recovery follow-up (2026-10-30, Python 3.12.13):

- Added broker-restart restoration of replay caller-ID/payload reservations.
  A previously reserved ID stays unavailable after confirmed STOP because the
  prior terminal response cannot be reconstructed. Malformed/truncated JSONL
  fails closed; a later durable confirmed simulated STOP restores framing and
  releases the global quarantine. Replay admission captures the SDK STOP epoch
  with its durable reservation, closing the reservation-to-dispatch STOP race.
- Regression coverage includes same-ID and changed-payload reuse after restart,
  truncated JSON and invalid UTF-8 journal fragments, and STOP between durable
  reservation and dispatch. No physical robot or hardware tests were used.
- `PATH="$PWD/.venv/bin:$PATH" .venv/bin/pytest --no-cov -q tests/test_simulated_replay.py tests/test_broker_replay.py` passed; `.venv/bin/ruff check .` and `git diff --check` passed.
- Full non-hardware run with coverage produced **869 passed, 1 failed**,
  **82.48% coverage**. The lone failure was the existing simulated CLI Sleep
  deadline test, which missed its 50 ms guard by 0.062 s under suite load.
  Rerunning that test alone passed; the focused replay suites passed afterward.
  No motion deadline or guard was relaxed.
- `.venv/bin/mypy src/soarm101_motion/broker.py --follow-imports=silent` passed.
  Broader project mypy remains limited by the previously recorded NumPy-stub /
  Python 3.10 target incompatibility (`numpy/__init__.pyi:737`); mypy is not
  part of this change's required lint gate.
- No hardware, live robot service, physical arming, commits or pushes occurred.

M2 simulated CLI Sleep timing follow-up (2026-10-30):

- Regular CLI `--simulation` now selects the deterministic non-real-time backend,
  matching `SimulationBackend`'s documented behavior. `sim-demo --realtime`
  remains the explicit wall-clock-paced simulation path. The controller's 50 ms
  command lateness guard is unchanged for real-time backends.
- `PATH="$PWD/.venv/bin:$PATH" .venv/bin/pytest --no-cov -q` completed with
  exit code 0; 870 tests were collected and passed. `.venv/bin/ruff check .`
  and `git diff --check` passed. No hardware tests or hardware were used.
- `.venv/bin/mypy src/soarm101_motion/cli/main.py --follow-imports=silent`
  reported 73 existing diagnostics in other CLI code (dynamic config kwargs,
  result unions, tool protocols and NumPy sequence annotations); it reported no
  diagnostic at the changed simulation selection. Scoped typing remains
  non-clean due to those repository baseline issues.
- No hardware, physical arming, commits or pushes occurred.
