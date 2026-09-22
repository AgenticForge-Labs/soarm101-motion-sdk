# Leader → follower teleoperation

Live teleoperation is implemented through the same guarded motion stack used by the rest
of the SDK, but it is deliberately rate-limited during initial hardware validation.

## Current conservative design

The normal planned-trajectory command clock remains **50 Hz**. Live teleoperation has a
separate explicit stream rate because the current Feetech safety path performs synchronous
serial feedback work for every accepted follower sample.

The GUI offers:

- **5 Hz — first hardware tests**
- **10 Hz — conservative default**
- **20 Hz — experimental**
- **50 Hz — experimental**

The SDK default is 10 Hz. Rates above 10 Hz require an explicit GUI confirmation on
hardware and are not considered physically validated.

Each streamed follower sample still performs the normal safety work:

- calibrated/model joint-limit validation;
- maximum command-step validation;
- rate-aware joint speed and acceleration validation;
- workspace path validation;
- hardware connected/torque/fault checks;
- measured following-error and opposite-direction checks; and
- managed-backend motor effort/current checks.

The selected stream frequency is used in the velocity/acceleration math. Lowering the
stream timer without changing this clock would be incorrect because the same angular
delta represents a lower physical velocity at 5 or 10 Hz than at 50 Hz.

## Backlog protection

Serial latency must not silently turn live teleoperation into delayed playback.

The GUI worker therefore measures each leader sample's age before execution. A sample is
rejected and the follower is held if it is older than the larger of 150 ms or three
selected stream periods.

Follower processing time is also measured. If processing takes longer than the selected
period for three consecutive samples, teleoperation is terminated and the follower is
held. The Teach status line reports approximately:

```text
Live teleop 10 Hz — 25 samples; follower cycle 42 ms; queued age 3 ms.
```

These are guardrails, not proof that a rate is suitable for a particular USB adapter,
host, firmware revision, or safety configuration.

## Hardware validation ladder

Do not jump directly to 50 Hz.

### 1. 5 Hz

Use Relative / clutch-safe mapping, no payload, and initially disable gripper mirroring.

Validate:

1. no movement at stream start;
2. small one-joint leader motions map in the correct direction;
3. follower cycle time remains well below the 200 ms period;
4. queued sample age remains low and does not trend upward;
5. STOP/HOLD terminates the stream immediately enough for the bench test;
6. disconnecting/stopping leader readout terminates follower teleoperation;
7. joint-limit, command-step, speed/acceleration, workspace, following-error, and effort
   trips fail closed.

### 2. 10 Hz

Repeat the same tests. The period is 100 ms. Record cycle times and sample ages during
single-joint and coordinated slow movements.

Treat 10 Hz as the initial practical target, not as a guaranteed production rate.

### 3. 20 Hz

Only try this after 5 and 10 Hz are repeatable. The period is 50 ms, so the current
per-sample serial monitoring may become the limiting factor. Any repeated overrun, stale
sample, growing queue age, or noticeably delayed STOP response means the rate is not
acceptable with the current transport path.

### 4. 50 Hz

50 Hz is the eventual high-rate target, not the current hardware assumption. The period is
only 20 ms. Do not call it supported on physical hardware until measurement shows that the
complete follower cycle can reliably stay inside that budget with adequate margin.

## What to record while increasing the rate

For each physical test rate, keep a small validation table with:

- USB serial adapter / device path;
- baud rate;
- rate requested;
- follower cycle time: median, p95, and maximum;
- leader sample age: median, p95, and maximum;
- count of processing overruns;
- stale-sample stops;
- communication errors;
- following-error trips;
- effort/current trips;
- observed STOP/HOLD response;
- whether gripper mirroring was enabled; and
- whether the test used one joint or coordinated motion.

A useful promotion criterion is not merely "it moved." There should be substantial timing
margin, stable queue age, no communication backlog, and repeatable stop/loss behavior.

## Likely optimization path

The current implementation intentionally prioritizes visibility and safety over maximum
rate. If 20–50 Hz is valuable after initial testing, optimize in measured steps rather
than removing checks.

### 1. Group/bulk reads

The largest likely improvement is replacing repeated per-servo register reads with
supported synchronous/bulk reads. Position, Moving/Status, Present_Current, and
Present_Load should be grouped where the Feetech transport and SDK support it.

### 2. Decimate expensive diagnostics

Joint targets still need validation on every command, but slower-changing diagnostics do
not necessarily need to be read at the command rate. For example, a future design could
write at 50 Hz while reading:

- positions/following state at 20–50 Hz;
- fault/status at 10–20 Hz; and
- effort/current at a separately validated rate.

Any decimation must include stale-data timeouts so cached safety state cannot be trusted
indefinitely.

### 3. Cache connection/torque state

Connection and torque state do not need a full six-servo status scan for every command if
the backend has a trustworthy cached state plus periodic verification and immediate
communication-failure handling.

### 4. Separate command and monitor scheduling

A future transport loop could maintain a deterministic command clock while a monitor loop
refreshes feedback into a timestamped cache. The command path would consume only feedback
fresh enough for its safety policy.

### 5. Benchmark before changing defaults

After each optimization, re-run the physical ladder and compare measured p50/p95/max cycle
times and queue ages. Raise the default only when the new rate has repeatable physical
evidence.

## Scope

This is host-side guarded position streaming, not a certified real-time controller.
Software STOP/HOLD is not an emergency stop. Keep physical power immediately accessible
during first hardware tests.
