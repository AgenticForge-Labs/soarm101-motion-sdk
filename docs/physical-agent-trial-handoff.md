# PR #94 — Trusted watchdog and attended physical agent trial handoff

**Status:** OPEN DRAFT. No real SO-ARM101 hardware has been exercised with this
new multi-action agent profile. Do not merge or enable the physical agent
broker as a normal service until local Codex/human evidence closes the gate.

## Boundaries

- Existing direct human CLI remains available and authoritative.
- OpenShell/robotctl/MCP agents never see raw servo, calibration or provider
  paths. Exactly one persistent SDK broker owns the serial device.
- For this *explicitly human-approved* trial, the only physical write tool
  is **jog_joint, at most 0.5° per request** in the supplied example;
  the server independently caps any physical profile to 1°.
- Defaults: 2°/s and 10°/s²; hard trial ceilings 6°/s and 30°/s².
- Real cameras, Cartesian jogs, gripper, Sleep and arbitrary saved poses
  are unavailable. The SDK still checks executable calibrated joint limits,
  robot identity, current human lease, servo faults and motor HOLD.
- Host and agent use **different bearer tokens**. Host private STOP,
  human authority revocation and a **5-second broker watchdog** are
  independent of LLM instructions. A lost trusted heartbeat latches STOP
  and cannot be rearmed within that broker process.
- A physical STOP is considered verified only when its SDK feedback
  confirms a stable encoder HOLD (not an HTTP 200 or an agent assertion).
  Software stop is **not** a hardware-certified emergency stop.

## Exact local Codex / human handoff

Repository: `AgenticForge-Labs/soarm101-motion-sdk`
Branch: `feature/sdk-broker-supervisor-watchdog`
Validation target: use the exact current PR head SHA and report it.
Sibling repos: NONE needed for SDK validation. External dependencies:
local installed OpenShell/Hermes/Codex and auth if testing those agents,
calibrated SO-ARM101 on explicit serial port, accessible physical cutoff,
working USB cable, current saved user calibration and human oversight.

### Step 1: no hardware

```bash
cd ~/AgenticForge/soarm101-motion-sdk
git fetch origin
git switch feature/sdk-broker-supervisor-watchdog
git pull --ff-only
git rev-parse HEAD
source .venv/bin/activate

ruff check .
python -m pytest --no-cov -q \
  tests/test_supervisor_watchdog.py \
  tests/test_broker_sdk.py \
  tests/test_agent_sandbox.py \
  tests/test_broker_stop_faults.py \
  tests/test_broker_session_lifecycle.py
python -m pytest --no-cov -q
soarm101 agent sandbox run --help
```

Do not continue if CI/local tests fail. Verify there are no unrelated
processes holding the serial device, and the local calibrated joint
ranges match the actual robot. Confirm the robot is unloaded, stationary,
with cables/desk swept clear and operator can cut physical power.

### Step 2: manual authority, bounded physical experiment

The **human**, not Codex/Hermes, issues motion authority first in a separate
local terminal. Confirm the current motor posture and reachable cutoff.

```bash
soarm101 agent arm --minutes 10
```

In a trusted interactive terminal:

```bash
soarm101 agent sandbox run \
  --agent codex --auth installed --interface mcp \
  --sdk-physical-supervised --serial-port /dev/ttyACM1 \
  --capability-profile docs/examples/physical-agent-supervised-joints.json \
  --task docs/examples/physical-agent-stop-trial-task.md \
  --output "runs/codex-physical-$(date +%Y%m%d-%H%M%S)"
```

The runner must require `PHYSICAL_AGENT_START` locally, confirm the
calibration-bound human lease and current SDK state, and keep the actual
device in its single persistent session. No automatic return to Sleep,
REARM or retry is permitted. If the arm behaves unexpectedly use the
physical cutoff, not just a software command.

### Step 3: interruption and lost-supervisor qualification

In a separately supervised run, arrange small bounded joint actions
and press Ctrl+C while an action is in flight. For watchdog fault
injection, a local Codex developer should first exercise fake/simulated
host heartbeat loss with tests and then use an attended, separately
approved trial; **never** casually kill or power-cycle the controller
while the arm is moving. Examine the order of: human agent action,
STOP admission, raw 0/254 HOLD, encoder settlement, cancelled action,
authority revocation, fault state and broker teardown.

Inspect the output directory:
`broker-events.jsonl`, `broker-events.motor.jsonl`,
`supervisor-stop.json`, `run-metadata.json`, relevant agent stdout,
and `openshell-effective-policy.yaml`. Preserve raw files unchanged.
Redact broker tokens and login credentials before sharing. Report
commanded vs measured motion, worst post-STOP drift, whether any
goal was written after STOP, and whether the test was truly in-flight.

## Merge gate

Before making the physical SDK broker normal for Hermes/Codex:
1. Final exact-head CI green on Python 3.10 and 3.12.
2. Local software tests green and a single SDK hardware owner confirmed.
3. Human-reviewed physical jog at the configured limits, no 1/1 rearm latch.
4. Attended Ctrl+C with verified independent STOP, <=1° post-latch
   settling, no post-STOP goal, revoked authority and no auto-rearm.
5. Loss-of-heartbeat fault injection validated, including its failure mode
   if servo communication itself is lost.
6. OpenShell ingress and broker token isolation verified locally.
7. Only then a separate explicit deployment/cutover action; this PR does
   not change running systemd services, .env files or production hardware.

The human direct CLI and bounded agentic control should ultimately
share the same deterministic SDK primitives. Do not certify arbitrary
physical speed/load/workspace or describe this software watchdog as
a rated emergency-stop device.
