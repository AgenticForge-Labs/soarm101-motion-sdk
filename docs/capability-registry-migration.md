# Shared SDK capability registry and broker migration

**Status:** PR #86 and registry/ordinary-CLI PR #87 are merged into
`main` (registry merge `e941e2686d2fd87569cdf1e91d6e508b228787ee`).
PR #88 is an open **draft**, not on `main`. It stages a persistent SDK
session through an opt-in **read-only simulation** broker and internal
fake/simulation HTTP tests for profile enforcement, concurrent STOP,
queued-request invalidation, lease revocation and camera evidence.
The deployed broker **still invokes the bounded `soarm101 agent` CLI**.
Physical device exclusivity, bus-level interruptibility, calibrated motion
and supervised STOP/2 mm trace validation remain merge blockers.
Dependent PR 3 (retiring agent-only wrappers) is not started.

## Purpose and ownership

Replace the duplicate agent-only motion execution surface with **one deterministic SDK
operation contract** reused by the ordinary human CLI and the broker. MCP and robotctl
remain constrained clients of the broker, not alternative motion authorities. The GUI
continues to call the SDK through its own persistent session. No Inspect Robots policy,
LLM, OpenShell sandbox, or evaluator may obtain direct servo access.

```text
human CLI --------------------------> typed SDK actions -----> guarded SOARM101
GUI --------------------------------> existing SDK APIs -----> guarded SOARM101
agent policy / OpenShell
  -> MCP or robotctl -> authenticated broker
  -> frozen per-run profile + human authority + audited dispatch
  -> typed SDK actions (one serialized robot session)
  -> guarded SOARM101
```

The diagram is **target architecture**, not the current call graph. The SDK remains
authoritative for calibration, tool/TCP, kinematics, trajectories, physical workspace,
joint and dynamic limits, following-error, faults, effort, cancellation, and HOLD.
The broker may deny or **narrow** capabilities, never relax those guards or independently
reinterpret physical geometry. Reasoning agents propose actions; deterministic code
validates and executes them.

## Registry contract to establish in PR 1

Use a small, explicit set of versioned, typed action specs mapping to existing SDK
operations. A spec must identify: stable action name, human-readable description,
effect class (read, motion, operator-only/administrative), typed parameter definitions
with units/defaults/bounds, stable result shape, and a deterministic SDK dispatcher.
Keep registration/introspection pure: listing actions must not open serial devices,
enable torque, connect cameras, load authority, or command motion.

Start with actual read/state, single-joint, linear/Cartesian, saved-pose, gripper,
camera-capture, STOP/HOLD, and diagnostic primitives already implemented; do **not**
invent new actuator abilities. Preserve existing ordinary CLI command names and output
behavior by treating the CLI as a parser/presenter of shared SDK operations. New registry
action names need not copy historical `soarm101 agent` command spelling. Internal SDK
values remain SI where applicable; CLI, broker and MCP conversion into human-facing
degrees and millimeters must be explicit and tested.

Do not put per-run authorization inside the registry. Operator-only operations
(calibration, arming, disarming, raw servo setup, relaxation and configuration where
applicable) must **never become agent-executable** merely because a registry spec
exists. Do not expose arbitrary Python method names, CLI argv, shell execution,
raw serial writes, or unrecognized action parameters as a generic dispatch capability.

Before replacing any existing route, characterize the present outputs and errors in
hardware-free tests; take behavior from today's SDK/CLI, not documentation assumptions.

## Broker dispatch and session migration in PR 2

1. A trusted operator selects a capability profile at startup; validate once, pin
   its canonical digest to the run, and enforce it **inside broker dispatch**, not
   solely through the advertised MCP tool list. Retain versioned profile examples.
   Profiles intersect with SDK safety; unknown actions, extra fields, unsupported
   camera names, invalid types/units and forbidden routes fail closed.
2. For each consequential call, use **current measured** robot state and current
   calibration/workspace identity. Recheck authority expiry, physical displacement,
   joint limits and effective rate caps as close to execution as possible. Cached
   capabilities, model FK or earlier preflight may not authorize a later motion.
3. Replace broker-spawned `soarm101 agent` subprocesses with one trusted, serialized,
   persistent SDK device session. Preserve robot ownership and disallow concurrent GUI,
   ordinary CLI or second broker hardware control of that device. Explicitly specify
   reconnect, stale calibration, disconnected camera and session-shutdown behavior.
4. Preserve the existing authenticated HTTP/JSON responses, MCP and robotctl contracts,
   trusted camera byte hashing, event/request IDs, error evidence, and redaction. Agent
   responses must not reveal host capture paths, device nodes or credentials.
5. **STOP/cancellation is a separate design gate:** the current subprocess broker holds
   an operation lock during synchronous motion, including its STOP route. Do not simply
   move that lock around a long-running SDK call. Design an interruptible motion path
   where an authorized STOP can signal the active session without waiting for the
   moving call to release the dispatch lock, while preventing concurrent ordinary
   commands to the serial bus. Define how HOLD completion, failed HOLD, timeouts,
   lease expiry and disconnects fail closed. Software STOP is never a substitute
   for accessible physical power cut-off.
6. Preserve the precisely scoped stale-Cartesian-start retry (full re-plan and
   workspace revalidation, bounded attempts; no execution of stale plans). Never
   broaden retries to motor faults, following-error, workspace or other execution
   failures. Keep the passive raw-goal/HOLD trace available through a trusted operator
   diagnostic that does not grant agent-supplied host file paths.
7. Treat physically unverified model coordinates as model estimates. Do not promote
   model +Z to measured table clearance, relax floor/IK checks, or enlarge agent
   Cartesian bounds to make a test pass.

**Minimum automated PR 2 evidence:** fake-transport parity with previous accepted
operations; reject unknown parameters/route aliases/profile escalation; two concurrent
requests serialize; authorized STOP interrupts an active move; STOP failure is observable
and leaves no false success; missing/expired lease or changed calibration is rejected;
workspace/current-pose TOCTOU is revalidated; session ownership/reconnect and camera
hash evidence are preserved; errors have stable externally visible classifications.

**Physical gate before PR 2 merge:** supervised, unloaded, externally observed 2 mm
Cartesian trial at already validated conservative rates, with existing calibration
and physical-clearance prerequisites verified first. Compare command, encoder, raw HOLD
goal and post-HOLD drift traces against the PR #86 subprocess baseline; do not infer
real clearance from the model. Stop immediately on an unexpected physical direction,
table approach, jerk/shake, command discrepancy or failed safety guard.

## Retiring agent execution wrappers in PR 3

Remove duplicate `soarm101 agent` motion methods and its broker subprocess executor
only **after** the direct-SDK path proves equivalent and safe. Keep the ordinary human
CLI, explicit human-only authority and administrative operations, relevant trusted
trace diagnostics, and the external MCP/robotctl contracts. Migrate OpenShell callers,
profile examples, help, README, AGENTS, testing, and safety documentation together.
Remove stale statements that the broker shells out to an agent CLI. Ensure no hidden
second implementation of rate limits, pose semantics, calibration or STOP remains.

## Merge order and independent evaluation

- First: PR #86 green CI **and** review of its local trace/physical gate, then merge.
- Second: PR 1 (registry/ordinary CLI parity), CI/test, merge.
- Third: PR 2 (broker direct dispatch/persistent session), CI plus physical safety
  validation, merge.
- Fourth: PR 3 (retirement/cleanup), CI and contract regression, merge.

Do not create dependent branches from `main` ahead of these merges.
Inspect Robots and Forge Bench are prospective **consumers/evaluators** of
capabilities, observations and evidence. They own policies, tasks and scoring;
they must not become another controller, calibration authority or motion-safety
implementation. Future benchmark integrations can compare policies and capability
subsets against the same pinned broker profile and SDK action semantics.

## Validation record to preserve

Record exact repository/commit, robot and calibration identities, profile hash,
motion-envelope and requested rates, observed physical clearances/directions,
operator intervention, trace files and SHA-256 hashes, CI process exit statuses,
and any disagreement between model predictions and physical measurements.
No test marked simulation/fake-transport-only proves real-arm safety.
