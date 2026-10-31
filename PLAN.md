# Development plan

## Puppeteer-owned robot MCP transition (post PR #97)

PR #97 has **merged** its simulation asset catalog, pinned replay and
watchdog/STOP fencing into SDK `main`. Reconcile the duplicate SDK and
Puppeteer MCP interfaces according to
[docs/puppeteer-mcp-migration.md](docs/puppeteer-mcp-migration.md).

Puppeteer's broker-native MCP and isolated client-bundle code is
merged in its PRs #4–#6. This SDK cleanup branch removes the
SDK-owned `soarm101-mcp` and its `[mcp]` extra, and delegates
OpenShell's client-only source staging to
`forge_puppeteer.mcp.openshell_bundle`. Native SDK, agent broker,
simulated replay, CLI/GUI and supervisor STOP are unchanged.
Cross-repository installed-wheel and OpenShell acceptance are
**still pending**; keep this change off production until qualified.


## Current roadmap status (2026-10-21)

The native SDK simulator, separate-token NVIDIA OpenShell relay and host-supervisor STOP were merged in PRs #90–#93. The current working change adds a broker-owned heartbeat/lease watchdog and opt-in attended native physical joint mode. Software fault-injection and simulated HTTP/subprocess tests cover the watchdog; general native physical control is **not hardware-qualified or the default**. The remaining milestone is locally attended encoder/STOP/watchdog qualification, followed by an explicit production deployment decision. Saved poses, gripper and Cartesian actions remain excluded from the experimental physical agent profile. This work has no assigned PR number.

The development entries below retain original experimental details and historical decision context. Where an older section calls a PR open/draft or assigns future work to “#94”, that is its **at-the-time status**, not the present merged or production state. Consult `README.md`, `AGENTS.md` and `docs/architecture.md` for today's architectural boundary.

## Simulation-only pinned replay package integration (2026-10-30)

Forge Puppeteer now has an isolated installed-wheel smoke at
`forge-puppeteer/scripts/soarm101-wheel-smoke.sh`. It builds wheels from
current Forge and SDK source snapshots, stages dependencies, installs into a
fresh offline Python 3.12 environment, and runs the actual SDK **simulation broker directly** through
Forge Puppeteer's installed MCP launcher (without a nested SDK MCP). The smoke verifies a
pinned sequence, stale revision rejection, and holding-confirmed simulated
STOP without loading sibling source trees at runtime. It does not exercise
recording replay separately: each replay reserves its identity until confirmed
STOP, and STOP ends that simulation session, so recording coverage needs a
second fresh broker session. No physical operation is part of this check.

The compatible SDK (`0.1.0.dev8`) and Forge bridge (`0.1.0`) are unpublished
artifacts. Locally built wheel metadata does not identify Git revisions, and
the default dependency staging resolves version ranges at run time. Reproducing
the exact build therefore requires retaining the copied source snapshots and
the complete dependency wheelhouse with hashes; the smoke is software evidence,
not a released dependency pin or deployment artifact.

## Historical implementation plans and qualification records

## #93 — OpenShell persistent SDK integration (merged; simulation only)

`--sdk-simulation` in the agent runner starts a host-owned persistent
simulated SDK controller with a distinct-token, bounded relay for
Hermes/Codex MCP or robotctl. The host supervisor owns the private
STOP path and invokes it before teardown. The default physical broker
remains CLI-backed. Hardware/multi-action SDK agent authorization is
NOT provided by #93.

**Next physical hardware milestone (not PR #94):** add broker-owned heartbeat/fault handling and
physically bounded multi-action profile, validate locally through Codex
with real joint encoder and STOP evidence, then decide on an explicit
configuration cutover. #93 is merged; no physical production service cutover was made.


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

With a local human-issued **simulation** authority lease, multiple
bounded simulated joint actions reuse one SDK session. STOP uses
the existing independent SDK dispatch, invalidates queued actions,
revokes the lease and writes broker events. STOP completion in
simulation is **not proof of physical servo braking**. This PR does
not change the OpenShell listener, start/stop live services, turn on
real-hardware POST, or use the trial's restricted physical capability
bypass. The ordinary physical broker does not have a `--sdk-physical`
mode. Real-world integration belongs to later #93/#94.

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

**Historical sequence:** #93 subsequently added the trusted OpenShell/MCP
transport and supervisor integration. Broker-owned watchdog, fault-injection,
supervised physical testing and any deployment cutover remain a separate,
unassigned milestone; they were not part of #92.


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


## 2026-10-11 PR #90 — reconnected live torque preflight repair

The attended Sleep-first short STOP script failed before motion:
`Sleep preparation requires a stationary, healthy torque HOLD`.
Root-cause source review established that `FeetechBackend.connect()`
did not recover the live `Torque_Enable` register state left by the
previous human `agent arm` process, so its new local cache could
incorrectly report torque off. The branch now reads all six real servo
torque registers on connect, rejects mixed/invalid states, and keeps
connection side-effect free. Joint-only physical Sleep does not
re-enable torque or send another 1/1 latch when already held. The
preflight logs exact flags and per-servo read-only diagnostics on failure.

Added software tests for new-session all-ON, all-OFF, mixed torque, no
goal write, joint-only Sleep reuse, and genuine torque-off refusal.
Keep the existing short STOP physical gate **blocked** until exact-head
software tests pass and the operator reviews hardware state. If moving,
faulted, or gripper-state issues persist, investigate original state
rather than bypassing the gate. No new physical STOP evidence yet.


## Draft PR #90 — prepared Sleep and actual in-motion evidence

2026-10-11 subsequent operator evidence shows a stable, single 0/254
HOLD and revoked lease in the short +8° pan trial, but **not moving-servo
braking**, because physical pan feedback was already stationary near the
endpoint before STOP. The same branch now stages the next test:

- All healthy, calibrated starting postures are eligible **only if**
  the operator confirms the entire multi-joint path to Sleep is clear.
- One trusted loopback broker session executes one **joint-only** canonical
  Sleep, verifies measured Sleep, then admits one fixed short pan motion.
  A broker-owned phase gate denies pan-before-Sleep, Sleep replay,
  and post-STOP motion. The production and public broker remain unchanged.
- A passive trace snapshot at the operator keypress must show recent
  changing encoder data, a fresh servo Moving flag, >=0.5° progress and
  >=2° remaining; otherwise the physical braking gate remains
  inconclusive even if stationary HOLD succeeded.
- No automated START/STOP, added hardware polling, agent authority,
  gripper close, auto-return or rearm. Local Codex/software validation
  and separately attended hardware evidence are still required.

Keep PR #90 draft/unmerged and the original 18.4° powered excursion
open as an unresolved physical safety blocker.

The motion SDK remains authoritative for hardware, calibration, kinematics, planning,
and safety. Forge Puppeteer owns higher-level physical performer/stage coordination;
historical Director/Studio integration references below describe older adapter work.

## PR #89 — Broker-owned SDK session lifecycle (merged)

Following staged PR #88 merged into `main` at
`957e86fb1f090a52960a593ce5b07caeaffb7ec6`, PR #89 hardens the
**opt-in SDK executor and existing Feetech device-locking implementation**.
The session is deliberately one-shot: lazy connection is permitted once, but
shutdown is terminal, and connection, calibration drift, serial faults, or
hardware faults latch a `faulted` state rather than reconnecting behind an
agent's back. Physical trial fault/shutdown revokes its human lease; STOP/HOLD
remains reachable against an *existing* arm while it is faulted or closing,
but STOP never opens a new device. Shutdown serializes with ordinary actions
and any active STOP. Serial open failure closes any partially opened transport
**before** releasing the cooperative lock.

This is **not** a production physical broker cutover. The normal
`soarm101-broker` stays CLI-backed; `--sdk-simulation-preview` continues
to deny physical and POST operations; the only real native-SDK entrypoint
remains the restricted operator-confirmed #88 trial. No automatic reconnect,
authority rearming, or failure retry is introduced.

Software gates: `ruff check .`, the new
`tests/test_broker_session_lifecycle.py`, real-tty advisory lock/failed-open
regressions in `tests/test_feetech_backend.py`, full suite, and simulated
concurrency/STOP regression. Hardware gates: local Codex/read-only
verification of one-owner GUI/CLI/second-broker exclusion and shutdown
with power cutoff accessible; no unintended motion. Complete physical STOP,
fault injection, native camera parity and 2 mm Cartesian qualifications belong
to proposed #90–#92. PR #89 merged into `main` at `77b28edba0f2beb129c031e6adf11513d0e240fe`
after Python 3.10/3.12 CI and operator-reported read-only local checks.

## 2026-10-11 physical STOP trace adjudication — FAILED safety gate

A complete source-level packet/firmware command analysis, original evidence
hashes, confirmed 1/1 new-goal defect, duplicate cancellation writers,
candidate 0/254 correction, and remaining hardware gates are maintained at
[Feetech STOP protocol investigation](docs/feetech-stop-protocol-investigation.md).
The same open PR now consolidates controller-owned cancellation, skips
idle-gripper STOP duplication, and prevents faulted shutdown from creating
additional position-latch commands. These changes are **simulation-gated
only** until verified with real motor feedback and an approved test plan.


User-supplied `stop-20261011-113611.jsonl`, `*.broker-events.jsonl`,
and `*.summary.json` document a real unattended 1-second post-STOP
joint drift despite HTTP 200 / `completed=true`. The operator correctly
reported acceleration and a subsequent return toward the commanded hold
posture. Broker STOP was requested at trace +62.899 s, ended at +62.922 s
(~24 ms), but the elbow moved from about **82.8°** then to **64.4°** by
+63.934 s (**18.4° further motion**), while the shoulder moved ~5.0° and
the model-derived TCP physical height changed from **51.1 to 111.6 mm**.
The position-latch raw writes at +62.907, +62.910 and +62.913 s all used
`speed_raw=1, acceleration_raw=1` and position ~2990 ticks. The last
ordinary streamed goal was +62.896 s, speed 0 / acceleration 254.
**No new ordinary motion command appears in the intervening second.**
By +92.13 s before the operator's separately authorized move, elbow
feedback was again ~83.3°: this suggests a motion reversal around the
latched position, but sparse feedback cannot determine the peak or exact
physical cause. Motor acceleration/braking dynamics at the servo are
**suspected**, not proven. The previous trial's `passed=true` result
is explicitly REVOKED as a physical STOP safety qualification.

Remediation in the same open PR: fail-closed physical STOP acceptance:
sample measured joint positions over 1.5 s after latch, reject >1°
post-latch drift or ongoing motor `Moving`/fault/torque loss, latch
`faulted` and keep authority revoked; expose stability evidence and
HTTP failure rather than claiming `completed=true`. Do not repeat a
failed/unverified hardware STOP command automatically, or offer the
post-STOP REARM path on failure. Fake-transport tests replay the
observed elbow displacement. This **detects** unsafe HOLD; it does
not repair or certify servo braking. Keep PR #90 draft, do not
activate native production motor paths or resume real STOP qualification
until Feetech STOP register behavior (especially 1/1 vs prior 0/254)
has been investigated with local Codex and a physical safety plan.

## PR #90 — Independent STOP response and fail-closed HOLD (open, unmerged)

Built on the #89 terminal broker session, this PR reinforces the existing
shared SDK STOP path rather than adding an alternate hardware controller.
`SOARM101.stop()` now attempts both motion-controller HOLD and gripper STOP,
preserving the original failure if either fails. The trusted native broker
serializes ordinary actions while dispatching STOP independently. After SDK
dispatch, the broker rechecks the original request's STOP epoch, refusing a
late success result for an action interrupted by a newer STOP. STOP that fails
in any way after reaching an existing SDK session latches a fault and revokes
physical-trial human authority; no automatic reconnect or resume is allowed.
A STOP success requires live connected/nonfaulted state and joint readback,
rather than unverified last-commanded positions. A successful simulated test
is not a physical STOP guarantee.

The reusable `examples/persistent_broker_stop_trial.py` adds a
separate trusted-host qualification: **verify measured Sleep without
commanding it**, complete Overhead -> Sleep and verify both positions,
then initiate a new Overhead movement and **wait for operator ENTER
while motion is active** before issuing independent STOP. Human
ROUNDTRIP/START/ENTER/STATIONARY gates, immutable bounded HTTP broker,
finite STOP timeout, and trace/event/summary evidence remain active.
STOP revokes the original lease and leaves the arm between poses.
The attended operator can decline recovery or explicitly type `REARM`
after verifying stationary HOLD. Only a new short calibration-bound
lease plus a new joint-only broker command can finish Overhead; the
cancelled action is never resumed automatically. Automated test
`tests/test_broker_stop_trial.py` checks no initial no-op Sleep,
measured pose tolerances, command order, and unattended rejection,
never serial activity. Actual physical acceptance remains operator-run.

Software gates: Ruff, `tests/test_broker_stop_faults.py`, existing
`test_broker_sdk.py` STOP/queued/HTTP tests, full pytest and both supported
CI Python versions. Physical gate: human-supervised active-motion STOP via
the restricted trusted-host loopback broker, with unloaded calibrated arm,
power cutoff available, timestamped independent STOP response, motor and
broker traces, no unintended resumed motion, and operator observation.
An error or incomplete HOLD is a BLOCKER; do not treat fake STOP or read-only
Codex success as completing this qualification.

Production remains bounded CLI-backed and the public persistent-SDK preview
remains read-only simulation. No interactive OpenShell physical motion is
enabled. Future #91 (native capability parity) and #92 (physical cutover)
must not branch until this PR merges.

## 2026-10-10 operator gripper fault and joint-only tracing

A supervised Sleep/Overhead/Left/Right/Sleep run reported Feetech
`so101_gripper` overload status 0x20; visual inspection alone cannot
determine whether the fault was sustained overload or a prior protection
status. The normal SDK correctly aborted motion, but the old hardware
STOP/HOLD read all six motors before requesting a hold. An unreadable
gripper therefore blocked the arm-motor hold command, and cleanup hid
the original failure. PR #88 now attempts live-position HOLD on each
readable motor, still reports partial failure, always tries disconnect,
and preserves the initiating exception. The route gains `--joint-only`
to avoid the post-Sleep gripper close during arm-joint characterization.
These software fixes require local hardware validation. Motor protection
is not suppressed. A separate trusted operator-only loopback bench experiment
is now available; physical persistent SDK execution is still not enabled for
the production broker or OpenShell agents.

## 2026-10-10 persistent physical broker route experiment (merged PR #88)

The trusted operator-only `examples/persistent_broker_motion_trace.py`
exercises five existing poses through the **real HTTP broker** with one
persistent SDK session: Sleep → Overhead → Left → Right → Sleep.
It uses joint-only requests by default (the earlier gripper overload
was transient), pinned joint speed/acceleration rates, a
calibration-bound human lease, a profile restricted to state, pose,
Sleep and STOP, and a loopback-only bearer-token service.
Evidence includes a session UUID, SDK connection count, passive motor
trace, broker event JSONL, and summary. A cooperative Linux tty
file lock prevents another updated SDK process from opening the same
real serial port, though older uncooperative software remains a risk.

**The supervised five-leg joint-only route completed on real hardware on
2026-10-10**, with one SDK connection, 1,677 commands, 444 feedback samples,
and five completed legs (operator-supplied JSONL trace, broker events and
summary). This narrowly validates that route, not interruptible physical
STOP, cross-process fault recovery or OpenShell agent motion. The experiment
does not authorize agent/OpenShell motion or the general physical SDK broker.
Production continues with the existing bounded CLI. The open merge
gates remain physical STOP/HOLD, device exclusivity, fault recovery,
camera/trace parity, and supervised calibrated Cartesian movement.
Retain the operator's trace evidence on merged #88. Additional physical
safety qualification belongs to subsequent PRs and must be attached to those
reviews rather than presented as already released behavior.

## Milestone B integration sequence (#88/#89 merged; #90 open)

GitHub `main` is authoritative. Do not create a dependent branch until the
preceding PR is merged. Preserve the extensive #88 staged work rather than
starting a competing SDK executor.

1. **PR #88 — staged SDK/broker foundation (merged):** complete software
   CI and local read-only OpenShell/MCP preflight passed. Production
   still uses the bounded CLI subprocess broker; the public SDK preview is
   read-only simulation. The operator-only physical route requires a terminal,
   human arming, explicit RUN confirmation and a separate narrow loopback profile.
   The completed 2026-10-10 five-leg experiment is evidence for *that route*
   only, not generalized robot safety.
2. **PR #89 — exclusive hardware ownership/session lifecycle (merged):**
   one broker-owned persistent SDK session; fail-closed disconnect, restart,
   authority expiry and calibration changes; confirm GUI/CLI/second-broker
   exclusivity, including cooperative lock limitations.
3. **PR #90 — independent STOP and fault handling (open):** true active-motion
   cancellation, generation-based queued-request invalidation, incomplete HOLD
   and servo faults; hardware-free race tests followed by a supervised STOP
   while moving with physical power cutoff accessible.
4. **Proposed PR #91 — native registry dispatch and camera evidence parity:**
   bounded joint/Cartesian/pose/Sleep/gripper/camera operations through the
   shared SDK; retain MCP/robotctl contracts, immutable profile, human lease,
   camera SHA evidence and complete auditable errors. Reject any duplicate
   untrusted host paths or direct agent hardware entrypoints.
5. **Proposed PR #92 — supervised physical qualification and cutover:**
   conservative 20 Hz trial candidate, observed 2 mm calibrated jog, STOP,
   stale position/workspace checks, fault recovery, camera capture and
   attended Codex/MCP run. Switch the production broker only after the
   documented physical gate passes. Do not infer clearance from model FK.
6. **Proposed PR #93 — retire duplicate agent CLI execution paths:**
   remove only the superseded bounded-agent motion executor and duplicate
   code; retain the human CLI, calibration/diagnostics and stable external
   capability contracts, update source-controlled architecture docs.

These PR numbers are tentative. This sequence **supersedes the original
three-PR cutover schedule below**, but keeps its original acceptance gates.
After Milestone B has merged, implement the persistent interactive Codex/Hermes
OpenShell terminal separately. Read-only interactive tests may be designed
earlier, but must not create a parallel robot controller. Ordinary real robot
usage is not a replacement for fault-injection and independently observed
STOP/ownership testing.

## Next milestone — interactive, sandboxed Codex/Hermes (design only)

The existing `soarm101 agent sandbox run` executes a one-shot harness process
(`codex exec --json --ephemeral` for Codex) then tears down its sandbox
and trusted broker. Running the interactive host `codex` with MCP is *not*
equivalent to OpenShell confinement: host tools can discover uncontrolled
device, subprocess, or network routes even when MCP tools are profile-limited.
Do not treat a prompt's instruction to stay within MCP as a security control.

**After the Milestone B production-broker safety gates are merged**,
implement a persistent interactive OpenShell terminal as a dedicated
follow-on change. Reuse the packaged Codex/Hermes adapters and current
MCP/broker/lease/profile primitives; never invent a second robot transport.
Proposed CLI UX (not yet implemented): `soarm101 agent sandbox interactive
--agent codex --auth installed --interface mcp --capability-profile FILE`,
plus explicit attach/status/close, selectable read-only or human-authorized
control, and predictable teardown/evidence. Preserve a single broker-owned
device session across prompts, with no host Codex shell fallback.

Acceptance requirements: per-session immutable broker profile/digest and
one-time bearer-token handling; sandbox unable to reach serial/camera nodes,
unrestricted SDK/host CLI, Docker socket, host code tree or unrelated credentials;
MCP tool denials enforced in the broker even when an agent attempts direct HTTP
or alternate shell paths; explicit human arming outside sandbox; lease expiry
and broker loss fail closed; independently interruptible STOP/HOLD; authenticated
named camera capture with SHA evidence; interactive attach/reconnect and
process cleanup without leaving torque or open hardware ownership ambiguous.
Use fake OpenShell/broker tests and real read-only Codex/Hermes interactive
trials *before* supervised physical motion. A narrow shipped joint-only
profile should be easy to select in one command, not require editing SDK code.
Physical validation and independent human power cutoff remain mandatory.

## Shared SDK registry and broker migration plan

**PR 2 work in progress (branch `feature/broker-persistent-sdk-session`):**
PR #87 merged into `main` at `e941e2686d2fd87569cdf1e91d6e508b228787ee`.
The new `broker_sdk.py` contains a persistent SDK session and strict typed
action adapter, initially exposed through an explicitly opted-in
`soarm101-broker --sdk-simulation-preview` **read-only simulation mode**.
In the public preview, `GET /v1/state` and `GET /v1/capabilities` use the
SDK session and **all POST routes remain denied**. An internal,
explicitly simulated test-only service setting permits HTTP regression
coverage for pinned profiles, expired authority, serialized commands and
concurrent STOP. A native STOP also invalidates already-queued requests
and revokes the simulated motion authority even if HOLD fails. It is not
a CLI option and cannot activate a physical SDK session. Production broker
operation still uses the existing bounded agent CLI subprocesses by default.
The preview intentionally does not authorize real hardware; tests of the
session adapter do not establish STOP interruptibility at the physical bus,
cross-process exclusivity, calibrated jog safety, or live reliability.
The internal fake-capture tests also verify image hashing, response
redaction and failure evidence without opening a camera device.

**Original PR 2 production cutover blockers (now sequenced across proposed
PRs #89–#92):** before replacing the production broker, complete
profile/lease enforcement at execution, direct SDK HTTP motion and evidence
parity, reliable concurrent STOP independent of the operation lock, shared
cross-process hardware ownership with ordinary CLI/GUI, session fault/
disconnect handling, simulated/fake-transport regression, and a supervised,
traced physical 2 mm jog with clear operator observation. Keep all fixes on
the sequential follow-up PRs; no new dependent branch before its prerequisite merges.



**Implementation update:** PR #86 merged into `main` at
`dfa2cbd128cf6cb4e37f9742333a63c0fc249cbc`.
PR #87 implements PR 1: typed SDK action registry, shared ordinary CLI
read/motion/IK/jog/gripper operations, saved-pose capture/replay, named
camera still capture, camera profiles, state and effort diagnostics.
Software CI passed on Python 3.10 and 3.12 with 586 tests; the GitHub
PR state is authoritative for whether #87 has merged. Operator-only
configuration, calibration, sequence/trajectory execution and the
agent-specific bounded subprocess remain deliberately unchanged.
The original PR 2/3 wording above is historical. The current staged #88
branch is implemented but unmerged; production cutover and wrapper retirement
are separated into the proposed #89–#93 sequence.


Detailed design, authority boundaries, STOP/cancellation acceptance tests, and
merge gates: [`docs/capability-registry-migration.md`](docs/capability-registry-migration.md).

**Status:** #86 merged and #87 implemented the first registry phase.
This original three-PR roadmap remains here for reference. PR 2 (trusted
broker direct SDK execution and persistent session) and PR 3 (agent-only
wrapper retirement) are **not implemented**. They must not begin from
`main` until #87 has actually merged; PR 2 also requires its own
separate real-arm safety/STOP validation.

**Target:** a single typed, deterministic SDK capability registry becomes
authoritative for supported motion/camera actions, descriptions, units, argument
schemas, SDK dispatch, and shared CLI/broker introspection. Human CLI and
broker/MCP are adapters. The broker owns agent allowlists, narrower per-field
limits, human-issued authority leases, request auditing and exclusive robot
session ownership. SDK guards remain non-negotiable independently of profiles.
Registry actions may be designated operator-only and cannot be enabled for a
remote agent by profile editing.

Implement as **three sequential follow-on PRs**:

1. **Registry and ordinary CLI parity.** Introduce typed action specs and
   adapters to existing SDK primitives; preserve human CLI syntax/semantics.
   Register existing read/state, single-joint, Cartesian, saved pose, gripper,
   camera, STOP and diagnostic operations. Cross-check parameters, units,
   descriptions and schemas against existing CLI; retain operator-only admin
   commands. Tests prove dispatch parity with today's SDK and no accidental
   hardware I/O during introspection. No broker runtime migration here.
2. **Broker registry dispatch and persistent session.** Broker validates
   static tool and parameter policies against the registry, performs **dynamic**
   calibration/physical-workspace/authority prechecks against current measured
   state, and directly invokes trusted SDK operations through one serialized
   long-lived device session. Preserve bounded retries, following-error, STOP,
   fault and cancellation behavior, camera evidence/sha, execution audit and
   OpenShell isolation; deny unknown arguments rather than forwarding CLI flags.
   Importantly, profile limits intersect with SDK safety and can only narrow.
   Test TOCTOU/revalidation, lease expiry, request concurrency, reconnect and
   failed HOLD, and supervised physical 2 mm jog/tracing versus subprocess
   baseline before merge.
3. **Retire agent-only execution wrappers.** Remove duplicate agent motion
   methods/subprocess execution while retaining human-only arm/disarm, regular
   operator CLI, generic trace diagnostics and stable MCP/robotctl contracts.
   Migrate existing profile examples, tests, docs and agent sandbox callers;
   verify there is no second authority source or motion implementation.

The broker should invoke Python SDK operations directly rather than shelling out
to unrestricted `soarm101` commands; exposing arbitrary CLI argv would be a
capability escalation. Generic registry metadata does **not** permit raw servo
register writes or arbitrary configuration from an agent. Do not widen policy
based on the unverified model/table geometry or treat model FK as true clearance.

**Precondition for PR 1:** #86 merged, local smoke/trace evidence reviewed,
and CI exits cleanly on supported Python versions. Current `main` remains
authoritative until then.

## Future PR 5 plan — composable agent/perception experiments (design only)

**Status:** intentionally not implemented. Begin experiments after the PR 4 MCP
sandbox interface passes local OpenShell/physical read-only validation and the
robot's measured coordinate mapping is independently checked. The motion SDK
must not become a perception engine, orchestration platform, or benchmark
runner. New perception providers belong in optional agent-side adapters; the
authenticated broker remains the sole robot execution boundary.

The experimental factor **agent/model setup** is broader than an LLM name.
Represent it as a versioned, explicit *pipeline graph* (components + interfaces +
configurations + provenance), including fully deterministic hand-coded baselines:

- A hand-coded finite-state controller using fixed cues and safety-bounded
  visual/pose corrections (no LLM inference);
- A hand-coded camera segmentation, feature-tracking, color/shape detection or
  fiducial pipeline feeding deterministic action selection;
- A small local detector or pointing/grounding model feeding deterministic
  planning with uncertainty gates;
- One general-purpose VLM controlling tools through MCP;
- An LLM planner + specialist vision model + deterministic target/guard
  converter + MCP execution;
- An inexpensive local observer with conditional hosted VLM escalation;
- Hybrid human-in-the-loop, scripted replay, and learned-policy baselines
  when they use the same authoritative validated motion interface.

Treat each as an **agent configuration**, not a new robot control plane.
Model identity, model build/hash, hand-coded algorithm revision/parameters,
pre/post-processing, perception model, observer/camera topology, reasoning
strategy, interface profile hash, safety gate, retry/escalation policy, and
human intervention must be independently identifiable. Keep provider and
inference cost distinct from local deterministic computation.

Candidate *read-only* perception outputs are timestamped image identifiers,
bounded image coordinates, bounding boxes, tracked object IDs, confidence,
uncertainty, and optional pixel grounding. Only a calibrated, validated
camera-to-robot transform can produce physical target poses; report missing
calibration as unavailable, not a guessed XYZ. Never let a detector directly
invoke motion or replace the Motion SDK workspace/IK/limit checks.

First supervised study: repeated cube-marker recognition, target selection,
coarse alignment, and re-observation using matched scenes and identical broker
constraints. Record visual evidence hashes, state, commands, rejected motions,
settle/error outcomes, wall time, inference tokens/cost, intervention/faults,
physical calibration provenance, and run configuration. Randomize/counterbalance
run order and scene arrangement where practicable. Forge Bench later owns the
experimental design, trial IDs, analysis and cross-run comparisons; do not
duplicate that engine in Motion SDK.

**PR 5 entry criteria:** physical coordinate calibration and read-only
image/tool plumbing characterized, benchmarkable control baselines defined,
clear ownership for specialist perception adapter, and an explicit safe
physical test protocol. Do not start implementation merely because PR 4
merged.

## Model, coordinates, and shaking audit — 2026-10-07

Reviewed main at `8830d9064bc6551674d1a7ec00c078a07ae73d12` and open motion
PR #79 at `583e4d554d7d7e898d83687cfb003af0ff6a7abd`. This visual correction is
independent of that PR; its motion changes must not be treated as merged behavior.

### Confirmed model and presentation findings

The planner and desktop GUI use native `SO101KinematicModel`, not a runtime URDF
parser. Optional PyBullet uses the bundled simplified URDF. Comparing 1,000 random
configurations against TheRobotStudio's official
`Simulation/SO101/so101_new_calib.urdf` gave maximum TCP position difference
0.00394 mm and orientation difference 0.000926 degrees. The reference file's last
modifying commit was `385e8d7c68e24945df6c60d9bd68837a4b7411ae`.
Nominal shoulder-lift→elbow, elbow→wrist-flex, and wrist-flex→wrist-roll origin
distances are 116.0, 135.0, and 63.7 mm. These are joint-origin distances, not housing
lengths. LeRobot's published new-calibration model contains the same chain/TCP
transforms. No evidence supports changing those link dimensions by guesswork.

- [x] Replace symmetric sliding-finger GUI geometry with a nominal fixed finger and
  rotating jaw using the official pivot/travel and mesh extents. This outline is not
  measured jaw angle or collision geometry.
- [x] Preserve scale across pose/target/ghost updates; provide explicit Fit, optional
  Auto fit, side/front/top/isometric views, and a screen-plane ruler.
- [x] Draw the active session TCP and compute numerical FK from the same joint snapshot
  as the drawing. Label the grid as model Z=0, not a measured table.
- [x] Add native/bundled-URDF parity, fixed-pivot, stable-scale, and active-TCP regression
  coverage. These checks establish software consistency, not physical accuracy.
- [x] Feed the persistent GUI model from measurements already owned by active motion/tool
  threads so commanded joint motion, model-estimated TCP readout, and gripper aperture remain
  live while the slower full-state poll is intentionally suspended. This adds no competing
  serial polling and no visualization authority over motion.
- [x] Replace the visual arm skeleton's coarse safety/joint-frame polyline with dedicated
  presentation geometry derived from the packaged URDF visual bodies. FK/URDF parity and
  coarse safety centerlines remain unchanged and separate. The offline illustration now uses
  a neutral 45% jaw opening and visually distinguishes the moving jaw from the fixed finger.
  Follow-up GUI review keeps body strokes deliberately thinner and flat-ended, removes the
  extra shadow, uses smaller pivot markers, and renders the moving jaw neutral dark and slightly
  heavier. A full presentation-assumption audit then compared the schematic against the
  official mesh-based SO-101 URDF at
  `385e8d7c68e24945df6c60d9bd68837a4b7411ae`. The GUI no longer assumes that a printed
  arm member starts/ends at each revolute-axis point: upper/lower members retain their official
  mesh-model Y/Z offsets, while each STS3215 is represented by an oriented 45.23 x 24.73 x
  35 mm case envelope at its official mesh pose. This makes the shoulder/wrist pivots sit
  inside actuator bodies where the physical model says they should, without inventing long
  shoulder/wrist bars. These shapes remain presentation-only and are not collision geometry.
- [ ] Add optional official CAD meshes with a switchable joint-center overlay. Preserve
  shared FK/tool authority, source revision/license, bounded rendering cost, and small
  window readability. Current schematic/primitive PyBullet visuals do not represent
  full printed housings or certified collision geometry.
- [ ] Display measured table/workspace overlays only with matching workspace and motor
  calibration provenance; do not label model Z=0 as physical ground.

### Physical coordinate investigation

Prior workstation reports include physical-to-model scales X≈0.829, Y≈0.927,
UP≈0.729; physical +50 mm D→UP produced model displacement approximately
(+11.5,+5.2,+30.9) mm. A model +Z hover also moved laterally and contacted the table.
These are reported hardware observations, not reproduced by this software audit.
Unequal scales cannot be corrected by rigid base-frame rotation/translation alone.
The cause remains unidentified.

- [ ] Collect read-only, torque-off joint/TCP correspondences at multiple heights,
  extensions, and wrist orientations using a repeatable physical base reference and
  one identified contact point. Repeat approaches from both directions to characterize
  backlash/compliance. Record raw/calibrated joints, calibration ID, active TCP,
  measurement uncertainty, payload, and model revision.
- [ ] Fit base alignment and TCP offset first; then assess joint-zero offsets, stop
  capture coverage, assembly indexing/variant, and independently measured link lengths.
  Encoder midpoint/travel calibration is not geometric calibration. Keep any geometric
  corrections separate from motor EEPROM calibration and preserve saved-motion/model
  provenance if a fitted model is adopted.
- [ ] Validate on held-out poses and multiple independent UP references. A four-corner
  plus one-UP affine fit has no independent vertical validation and may compensate only
  locally; do not promote its scale/shear to a global geometry repair.
- [ ] After measurement review, design supervised small-displacement physical-direction
  and straightness checks with before/after measurement. Preserve full-path preflight
  and physical guards. Lowering the floor, loosening IK tolerance, or substituting
  joint replay does not repair geometry.

### Linear motion and shaking investigation

The current Cartesian controller already samples the Cartesian line at command rate,
uses sequential IK with continuity checks and Cartesian-constrained smoothing, and
executes with responsive servo tracking `0/254`. No swapped-axis or mm/m conversion
error was identified in shared jogging. Software agreement does not prove physical
TCP straightness.

- [ ] Complete PR #79's outstanding supervised gates on its exact head before adopting
  its motion changes. The branch contains duplicate encoder-target coalescing,
  responsive ordinary joint tracking, less-folded default Sleep, and wrist-aware teleop.
  Prior reported final-target execution still rocked: repeated writes are not the sole
  cause. The taught sleep2 posture and later 80 deg/s, 500 deg/s² responsive-profile
  tests were reported substantially smoother; low-speed Cartesian shake remains
  unresolved. Finish existing branch validation rather than duplicating its changes here.
- [ ] Compare planned joint derivatives/reversals/quantized goals with measured joint
  position, following error, current/load, actual servo profile, and cycle timing on
  matched paths, postures, payloads, and cadences. Distinguish oscillating commands
  from oscillating mechanics under smooth commands. Compare taught teleop replay with
  Cartesian plans using the existing PR #79 diagnostic tools.
- [ ] Inspect wrist mounts/horns, fasteners, cable forces, backlash, gravity loading,
  and near-limit/folded postures. Position-only IK permits posture changes; report
  Jacobian conditioning and limit margins rather than assuming all shaking is IK.
- [ ] Consider servo tuning/path shaping only after traces identify a cause. Faster
  motion provides evidence about low-speed behavior, not a universal repair. Preserve
  joint, following-error, fault, effort, timing, workspace, and stop protections.


## Implemented motion foundation

- [x] Direct official Feetech SDK hardware backend.
- [x] STS3215 IDs, register table, model verification, and synchronized commands.
- [x] Native and LeRobot-compatible calibration storage/discovery with content-addressed
  calibration IDs, immutable history, and motion-artifact provenance.
- [x] Five-joint arm and `SO101Gripper` tool separation.
- [x] FK, bounded IK, minimum-jerk joint movement, and preplanned linear movement.
- [x] Blocking and nonblocking motion handles with unified cancellation.
- [x] Torque-safe enable, trajectory validation, feedback completion, timeouts, diagnostics, CLI, and simulation.
- [x] PySide6 Setup/Control/Teach workspaces with shared follower/leader GUI
  mechanical-stop calibration.
- [x] Persistent Home/Rest pose library, calibration-derived natural Sleep posture, and
  calibrated joint-control ranges.
- [x] Independent read-only leader/controller connection and selectable teaching source.
- [x] Named taught points captured from follower or leader and replayed as joint or linear moves.
- [x] Immutable 50 Hz trajectory recording with gripper and optional effort/current diagnostics.
- [x] Validated trajectory replay with safe pre-roll to the recorded start pose.
- [x] Trajectory Editor v1: timeline, scrub, crop selection, speed scaling, Save As, and replay.
- [x] Persistent sequence editor/runner with point, Home/Rest, gripper, wait, trajectory,
  and semantic primitive steps plus step execution, repeat, speed scaling, pause/resume,
  cancellation, and STOP/HOLD integration.
- [x] Guarded 50 Hz joint streaming and leader-to-follower teleoperation in relative or
  absolute calibrated modes, with optional gripper mirroring.
- [x] Advanced non-destructive trajectory editing: smoothing, delete/splice, holds,
  keyframes, markers, repeated clips, and semantic motion primitives.
- [x] Leader FREE/PARKED state, cross-arm pose matching, no-motion relative relink, and
  coarse leader → fine Manual transfer workflow.
- [x] Persistent Manual gripper panel and one synchronized GUI gripper-speed preset across
  manual, teleoperation, saved-pose, sequence, and trajectory-replay paths.
- [x] Position-first GUI Programs workflow over the existing MotionSequence engine, with
  saved-position Move steps, Open/Close/custom gripper actions, waits, per-Move speed,
  and advanced recorded-motion steps.
- [x] Persistent right-hand live follower sidebar across every GUI tab, with modern
  styling, current joints/TCP/gripper/status, always-available follower controls, and
  contextual ghost overlays for leader/saved/recorded/program poses.
- [x] Radial shoulder-pan Program pattern generator that inherits a saved base pose and
  overrides only shoulder pan across an inspectable series of guarded Move steps.
- [x] Shared USB-camera foundation with named persisted camera profiles, one GUI worker per
  physical device, concurrent multi-camera streaming, Camera/Teleoperation named views, and
  CLI discovery/configuration/named or all-camera still capture.
- [x] Shared machine-local workstation profile for follower/leader addressing, calibration
  references, and named cameras consumed by GUI, CLI, and external agents.
- [x] Model-agnostic `agent-as-code/` machine setup and multi-view capture helper plus a
  neutral SDK-level agent CLI contract.
- [x] Bounded `soarm101 agent ...` facade with human-interactive time-limited motion
  authority, `agent_*` saved poses, overhead/wrist capture, safe gripper endpoints,
  Sleep/STOP-HOLD, and measured-height-limited translational jogs.

## Director ecosystem integration 0.2

- [x] Add `.agenticforge/integration-manifest.json` and `INTEGRATION.md`.
- [x] Publish stable units, base-frame, resource, stop-classification, and TCP-ownership metadata.
- [x] Define this SDK as a library consumed by Studio and Puppeteer rather than a Director service.
- [x] Expose a shared `motion-platform:<id>` resource convention.
- [x] Document nonblocking handle and cancellation requirements for consumer adapters.
- [x] Add integration metadata tests.
- [ ] Validate the Studio SO-ARM motion-platform plugin in simulation.
- [ ] Validate the Puppeteer named-gesture embodiment in simulation.
- [ ] Define and validate the trajectory/demo-data adapter from this SDK's recorded
  leader/follower trajectories into Robo Puppeteer training and motion-data workflows.
- [ ] Pass the Director cross-repository simulated-show suite.

## CLI and GUI feature parity

The CLI and GUI must call the same SDK operations and saved libraries. The GUI keeps
its own persistent connection for live controls; it must not launch CLI subprocesses.

- [x] Match basic setup, structured readout, read-only IK, joint motion, Cartesian jogs,
  and gripper motion, including simulation-capable one-off CLI checks.
- [x] Add CLI arm discovery, absolute Cartesian moves, named pose capture/replay,
  trajectory replay, sequence execution, and effort status.
- [ ] Add CLI trajectory recording, non-destructive editing, and primitive management.
- [ ] Add CLI sequence and named pose library editing beyond capture/replay.
- [ ] Add CLI effort guard configuration, peak reset, and trip clearing.
- [x] Add CLI camera discovery, shared settings, and fresh still-frame capture.
- [ ] Add a persistent CLI session for STOP/HOLD, pause/resume, and guarded leader-to-follower teleoperation.
- [ ] Verify matching behavior in simulation and fake transport before hardware use.

## Physical validation gate

- [ ] Complete physical characterization of the explicit 100/1000 host motion envelope.
  The previously host-rejected **80 deg/s, 500 deg/s²** Sleep-vs-`sleep_up` comparison
  completed successfully on 2026-10-06 with responsive 0/254 tracking and was judged much
  smoother by the operator. The first **100/1000** attempt never moved because SI conversion
  differed from the exact configured ceiling by a few floating-point ULPs; that software
  boundary bug is now fixed. Re-run 100/1000 under the unchanged following-error,
  effort/fault, calibrated-limit, workspace, STOP/HOLD, and settle safeguards before
  marking the full characterization gate complete.

- [ ] Run port discovery and six-motor model check on the user's arm.
- [ ] Import or perform calibration and verify all directions.
- [ ] Run the conservative 2° CLI smoke test on each joint with no payload and verify
  physical direction before Cartesian motion.
- [ ] Validate stop, completion timeout, and torque-safe enable on hardware.
- [ ] Validate FK against measured TCP positions.
- [ ] Complete the paper/workspace calibration: four table corners plus a physically
  measured UP reference tied to the active motor calibration.
- [ ] Run the supervised paper traversal using the existing A/B/C/D + D_UP calibration:
  one calibrated-Z startup clearance move, then constant-height Cartesian
  `move_linear()` replay A_UP→B_UP→C_UP→D_UP→CENTER_UP at 20 Hz. Verify all targets
  share the trained workspace Z and compare physical smoothness directly with teleoperation.
- [ ] Diagnose the A_UP->B_UP straight-line feasibility before any further powered
  paper replay. Hardware #75 evidence shows forward and endpoint-seeded reverse IK both
  converge near the same residual (0.648/0.646 mm) against the unchanged 0.5 mm tolerance.
  Require read-only full-segment preflight of every elevated paper segment and inspect the
  failing sample's residual vector, Jacobian conditioning, and joint-limit margin to decide
  whether this is optimizer behavior or a real model-workspace boundary. Use the torque-off
  calibration-limit comparison first: the active mechanical-stop calibration is wider than
  the nominal model on every pose joint, including wrist flex ±103.9° versus ±95°. The
  torque-off margin search has now bounded the original 107 mm path's wrist requirement:
  5.430° from the measured stop is feasible and 5.469° is not. The boundary solution reaches
  +98.48° wrist flex against a measured +103.91° stop. Runtime limit authority now uses
  the URDF/model range as the generic fallback/reference, but a calibrated real arm follows
  its measured pose-joint travel with a 1° inset from each mechanical stop. This preserves
  nearly all measured range while keeping normal commands off the stop itself. Validate the
  normal runtime preflight and one supervised powered replay before treating the extended
  calibrated range as physically validated. Then use the paper-height sweep only as a
  separate geometry comparison. The
  first 5 mm sweep found 130 mm (+23 mm from
  the measured 107 mm reference) fully feasible, while candidates through 125 mm remained
  wrist-flex limited. Treat 130 mm as extrapolated calibration geometry until separately
  validated. Inspect its planned joint/encoder diagnostics before any powered use. The severe
  visible shake remains a separate
  unresolved issue; after planning is feasible, inspect planned
  joint derivatives, encoder quantization, following error, and cycle timing before changing
  motor tuning.
- [ ] Complete the guided teleop-versus-programmed motion-quality study on the physical
  follower. Capture a marked slow teleop reference, guarded replay of its exact accepted
  arm-joint commands, and the same 8 deg/s saved-pose route at both 50 Hz and 20 Hz. Compare
  raw encoder-step distributions, command-versus-feedback lag, deceleration behavior, and
  folded/Sleep geometry before selecting the next control change. Physical 20 Hz versus 50 Hz
  comparison showed no large smoothness difference, while exact deterministic replay of the
  recorded teleop command sequence completed successfully and retained the visibly less-
  mechanical/smoother character of teleop. This makes generated joint-command structure the
  leading hypothesis, especially during the folded return to Sleep. Quantized-target
  coalescing has looked at most marginally better and is not merge-ready on that evidence alone.
  Keep both joint execution strategies available while testing: the existing `streamed`
  host trajectory and the experimental `final_target` one-write endpoint mode. Run the
  supervised same-route comparison, with special attention to Right -> Sleep, before choosing
  a production default or removing either implementation. The first final-target physical
  attempt completed Sleep but was stopped on Sleep -> Overhead by an invalid cross-joint
  phase-coupling guard (0.308 rad vs 0.300 rad). That guard has been removed while preserving
  per-joint corridor/overshoot, reverse-motion, effort/fault, cancellation, timeout, and
  settle checks. Because the one-shot servos may trace an asynchronous joint combination
  rather than the synchronized host plan, full-workspace/saved-pose execution now validates
  the accumulated measured intermediate path on every monitor cycle using the measured-start
  workspace policy. The corrected final-target-only rerun completed the full route but still
  rocked substantially, so repeated host micro-waypoints are not the primary cause. Retain
  both execution modes for diagnosis. Faster requested speed is somewhat smoother overall
  and higher acceleration adds little beyond that, while severe rocking returns during the
  final slowdown into canonical Sleep. Before deeper servo tuning, isolate wrist
  orientation/fold order with a physically taught wrist angle: hold canonical Sleep, relax
  only wrist_flex, hand-place it, relatch/save the measured pose, then compare direct canonical
  Sleep, direct taught-wrist Sleep, and a staged final wrist-only fold from RIGHT. Return to
  teleop-derived command shape versus programmed low-speed servo behavior after that result.
  The operator-defined `sleep2` A/B showed substantially smoother settling than the
  historical Sleep pose, confirming a large geometry component. Promote that result into a
  portable calibration-relative semantic: default Sleep uses wrist_flex at 75% of its
  executable range, while `sleep_up` preserves the historical lower-limit wrist fold.
  Re-run the standard route with the new default Sleep on hardware before merging. A 40
  deg/s, 250 deg/s² attempt showed that the old backend 250/20 Feetech profile could no longer
  track the faster validated host trajectory and tripped following-error before the A/B.
  Default streamed planned joint motion now uses the same responsive 0/254 tracking profile
  as teleoperation while retaining the host speed/acceleration ceilings and safety guards.
  The 40/250 Sleep-vs-sleep_up rerun completed cleanly after removing the hidden servo
  throttle, and 80/500 subsequently completed with a large subjective smoothness improvement.
  GUI teleoperation then isolated a different wrist-specific issue: 20 Hz Medium tracking
  works, while the former Fast 1000 deg/s² wrist-flex response could command a reversal
  faster than the physical wrist braked, producing +0.057 rad of carry-through and a strict
  opposite-direction stop. Fast is now per-joint: wrist_flex retains 100 deg/s speed but uses
  500 deg/s² acceleration, while the other joints retain 100/1000. Live-stream direction
  monitoring also permits only a 100 ms non-growing braking carry-through immediately after
  a recent genuine command reversal, capped at 0.10 rad cumulative wrong-way travel;
  planned motion remains strict. Validate isolated wrist
  reversals at 20 Hz with no payload/gripper mirroring before considering this motion-quality
  branch merge-ready.
- [ ] Resolve any remaining physical/model Cartesian-direction mismatch before broader
  low-speed linear paths/tolerance validation.
- [ ] Finish guarded leader-to-follower streaming validation. Medium at 20 Hz has passed
  supervised use; next validate Fast · wrist-aware wrist reversals, STOP, leader-readout
  loss, and stream safety trips with gripper mirroring disabled, then separately validate
  gripper mirroring/contact at Normal gripper speed. The pen overload showed that gripper
  object state must not be conflated with arm-stream dynamics.
- [ ] Validate sequence execution and edited motion primitives on hardware.
- [ ] Validate the selected USB camera on the target workstation, confirm negotiated
  resolution/FPS/FourCC, live GUI preview in Camera and Teleoperation, and still capture.
- [ ] Tag the first hardware-validated alpha release.

## Later

## M2 simulated replay recovery follow-up (2026-10-30)

Durable replay reservations now restore caller request-ID/payload bindings on
broker restart. A replay ID with unknown outcome remains unusable after a
confirmed STOP; new IDs are admitted only after a durable confirmed-STOP
reconciliation record. Malformed/truncated journal records fail closed and a
later STOP record restores JSONL framing and clears the quarantine. Replay
reservation captures the SDK STOP epoch atomically, so a STOP between reservation
and dispatch cancels the queued replay. These guarantees are simulation/software
evidence only; durable terminal response reconstruction, clean-install/stdio
acceptance and physical replay remain unimplemented. Focused replay tests and
Ruff passed. The full non-hardware suite reported 869 passed and one
pre-existing timing-sensitive simulated CLI Sleep deadline miss (0.062 s beyond
its 50 ms guard); the test passed in isolation. Full-run coverage was 82.48%.
See `TESTING.md` for commands and tooling limitations.

- [ ] Revisit optional agent sandboxing after the bounded CLI/camera robot loop is
  physically validated. The bounded CLI is the robot capability boundary; a sandbox may
  further restrict shell/filesystem access but is not an architectural dependency for the
  initial experiment.
- [ ] Investigate true leader gravity compensation as a separate feature. Compare the
  Trossen leader/SDK approach before selecting a control strategy; do not couple this to
  the current position-hold parking implementation.
- [ ] Parallel gripper implementation.
- [ ] Tool assemblies with camera and gripper TCPs.
- [ ] Optional collision geometry and self-collision checks.
