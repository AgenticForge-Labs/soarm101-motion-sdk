# Trusted deterministic choreography capability (PR 3 of 5)

**Status:** New SDK capability proposed on `agent/deterministic-choreography-execution`. It is not part of `main` until GitHub reports the PR merged. Hardware execution, independent STOP qualification, calibrated clearance and final three-robot demonstration are outstanding. See [Safety](safety.md), [Architecture](architecture.md) and [Motion Programs](programs.md).

## Authority boundary

```text
Forge Conversationalist (LLM personality 4G and business tools)
          |
Forge Puppeteer (named gestures, emotional interpretation,
                  choreography authoring and loops)
          |
TRUSTED operator-selected Puppet SDK adapter (PR 4, later)
          |    - EXACT approved segment/revision
          v
SO-ARM101 Motion SDK (this PR)
  JointChoreography + ChoreographyPolicy
       -> read-only full-path preflight
       -> existing persistent SDK executor + supervisor watchdog
       -> SOARM101.play_trajectory (canonical motion controller)
       -> measured hardware feedback + STOP/HOLD
```

**4G is not a motion profile.** This SDK has no emotion, personality, voice, tool-agent or Conversationalist dependency. All robots may embody 4G simultaneously through Puppeteer in later PRs.

The **existing untrusted agent-as-policy** route remains unchanged, including the historical `AGENT_JOINT_MAX_DELTA_DEG` ±2° envelope. This module deliberately does **not** add a route to `_ACTION_FIELDS`, `BROKER_CAPABILITY_ROUTES`, `robotctl`, CLI or the SDK's old MCP executable. A model cannot create a trusted choreography grant by passing its own data.

The separately authorized choreography path is reusable by a trusted *human-owned* native SDK application and by the future Puppeteer operator-configured SDK adapter. It is neither a new motor driver nor an alternative emergency-stop system.

## Source contract

`src/soarm101_motion/choreography.py` exports:

- `JointFrame(at_s, joints_rad)`: one full five-joint, finite absolute target in canonical `ARM_JOINTS` order, radians, timestamp relative to program start. No gripper channel, action name, direct servo registers or Cartesian/world-relative motion.
- `JointChoreography.author(name, robot_id, calibration_id, frames)`: a finite reviewed segment, 2–256 keyframes, starting at t=0 and finishing in at most 30 s. The exact canonical content is identified by a `sha256:` revision; loading mutated/stale data is rejected.
- `ChoreographyPolicy(... allowed_revisions, max_duration_s, max_samples, max_start_error_rad, max_speed_deg_s, max_acceleration_deg_s2, allow_physical_trial=False)`: the trusted host's pin set, robot/calibration identity, duration, start and motion envelope. The policy limits are *additional*, never an SDK limit override.
- `preflight_choreography(arm, pattern, policy, speed_scale=1)`: **read-only** complete-path preflight. It checks the current measured start, exact revision/robot/calibration, SDK health/stationarity, retimes/resamples via `Trajectory`, and runs the canonical controller's per-sample joint-speed/acceleration validation and SDK workspace validator. It cannot silently move into the pattern's first pose or auto-enable physical torque.
- `execute_trusted_choreography(executor, watchdog, policy, pattern, speed_scale=1)`: requires an already-connected persistent `SDKAgentExecutor` and its **same** actively running `BrokerWatchdog`; no implicit device creation. Checks all identity, authority, lease, full path, and watchdog boundaries before motion. Calls the existing public `SOARM101.play_trajectory`, which performs a second fresh guarded preflight at execution, uses the SDK controller, and reports its outcome. It never opens another serial connection.

For physical trials, additional requirements are: `physical_actions=True` established by the *trusted operator* at SDK executor construction, explicit policy `allow_physical_trial=True`, active human lease bound to the real calibration, existing operator-enabled torque HOLD, an active watchdog, current SDK workspace checks, and available physical STOP/power isolation. No automatic torque enable on real hardware. These are **necessary but not evidence of physical qualification**.

For software CI, only the existing `SimulationBackend` is used; simulation can enable virtual torque after successful preflight. The physical path remains a supervised experimental one, not the default agent broker.

### Why this executes a complete path instead of chaining agent jogs

An authored 20 Hz sinusoid, nod, shake, or transition can have movement exceeding the ±2° untrusted jog envelope **only when it has been preapproved in its entirety**, pinned to a calibration, and passed the SDK's existing whole-path, rate and workspace checks. It is never implemented by chaining agent-policy jog requests or skipping SDK planning. Segment repetition and expressive meaning remain in Puppeteer, **not** in the SDK.

The SDK's ordinary recorded playback historically writes gripper goals at every sample. PR 3 adds a **default-preserving** `joint_only=True` option at the canonical `SOARM101.play_trajectory` and `MotionController.play_trajectory` boundary, and expressive execution always selects it. The authored trajectory's constant gripper channel records the measured start but the joint-only executor **never writes any gripper goal**. Ordinary recording replay keeps the existing default gripper semantics.

`preflight` and SDK completion are **not** proof of physical vertical direction, self-clearance in the real workspace, servo response, or a safety-rated emergency stop. The SDK's known history includes ~18.4° post-STOP elbow travel in an earlier physical trial. Keep hardware attended with readily accessible power isolation.

## Interruption and failure

The existing broker watchdog runs independently of ordinary motion dispatch. During execution, it fences command admission on heartbeat/lease loss, invokes the SDK-owned STOP/HOLD attempt and retains its terminal state. Each segment also has a bounded deadline; failure or uncertain completion invokes the existing `_stop_after_failure` path and raises `ReplayExecutionError` with separate STOP evidence. There are no retries, automatic reconnects or automatic recovery after unconfirmed outcomes.

The returned success message is intentionally limited to `sdk_execution_completed_physical_settling_not_certified`. Follow-up robot telemetry and independently measured hardware positions remain a separate local qualification.

## Usage by a trusted host (simulation only)

```python
from soarm101_motion.choreography import (
    ChoreographyPolicy, JointChoreography, JointFrame,
    preflight_choreography, execute_trusted_choreography,
)

# In a trusted host, use an ALREADY-CONNECTED persistent SDKAgentExecutor
# and its existing BrokerWatchdog. Do not construct a new arm/USB owner.
arm = executor._session
measured = arm.get_joint_positions().positions
names = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
start = tuple(measured[name] for name in names)

frames = (JointFrame(0, start), JointFrame(1.0, start))
pattern = JointChoreography.author("still-check", "so101", "simulation", frames)
policy = ChoreographyPolicy(
    robot_id="so101", calibration_id="simulation",
    allowed_revisions=(pattern.revision,),
)
report = preflight_choreography(arm, pattern, policy)
# This call is only available to the already-authorized trusted host:
result = execute_trusted_choreography(executor, watchdog, policy, pattern)
```

The final Puppeteer adapter in PR 4 must use the SDK's **canonical** segment and policy revision pins, not copy these checks into Puppeteer or provide agent-supplied policy objects. It must maintain one host SDK connection and the existing watchdog.

## Software acceptance and Codex handoff

GitHub CI must run Python 3.10 and 3.12, full `uv run pytest`, Ruff, compilation, wheel build and CLI/GUI smoke as configured in `.github/workflows/ci.yml`. The focused simulation tests must establish:

- exact content hash and calibration identity, finite strict timestamps, five-joint schema and bounded full path;
- no actual motion/torque during preflight; no implicit path-to-start;
- successful simulator execution through the existing SDK;
- no new untrusted broker/MCP route or widening of agent policy;
- denial for missing watchdog, unapproved revision, stale measured entry, rate increase and unqualified physical mode;
- failure and STOP evidence with no autonomous retry.

After all five software PRs, local Codex will validate the *actual* Overhead taught pose, verify modeled versus real +Z, supervise 0.1 Hz breathing and bounded wrist behaviors, capture encoder traces, verify STOP drift and any effects of the wobble/trajectory profile work, and certify only what the attended measurements support. Use exact merged SDK and Puppeteer SHAs in that handoff. Never issue an unattended hardware command as part of CI.
