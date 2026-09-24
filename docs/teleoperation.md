# Leader → follower teleoperation

Live teleoperation is implemented through the same guarded motion stack used by the rest
of the SDK. Its host-side target limiter remains active while streamed Feetech position
commands use the same unrestricted servo speed and maximum acceleration settings as
LeRobot.

## Streaming design

The normal planned-trajectory command clock remains **50 Hz**. Live teleoperation has a
separate explicit stream rate because the current Feetech safety path performs synchronous
serial feedback work for every accepted follower sample.

The GUI offers:

- **5 Hz — slow check**
- **10 Hz**
- **20 Hz — default**
- **50 Hz — experimental**

After both arms are connected, **Align follower and start** reads a fresh leader
pose, latches the follower's current positions before enabling torque, and moves
the follower's five pose joints to those leader angles with a guarded planned
move. With gripper mirroring selected, it then opens the follower gripper to
the leader's measured normalized opening if needed. Any closing difference is
handled by the live contact guard after joint alignment. Keep the leader still
during alignment. The button can cancel alignment, and STOP/HOLD also stops it.
Joint alignment uses a 25°/s planned speed and the responsive servo profile
used for live following; completion checks the five measured pose joints.
Absolute calibrated mapping is the GUI default; relative mapping remains available.
The gripper always uses the leader's normalized opening, even when pose joints
use relative mapping.
If a leader joint exceeds the conservative model motion limit but remains inside
the follower's recorded calibration range, alignment stages 0.5° inside the
model limit and switches to relative mapping. The GUI and session log name the
resulting offset. A leader joint outside the follower's calibrated travel is
rejected before torque enable or motion.
The Setup **Enable hold** control remains useful for Manual motion. Manual joint
targets follow the measured follower until **Edit joint targets** is selected.

The SDK default is 20 Hz. Rates above 20 Hz require an explicit GUI confirmation on
hardware and are not considered physically validated. The 20 Hz default has not yet
been physically validated on this arm.

Each streamed follower sample still performs the normal safety work:

- calibrated/model joint-limit validation;
- maximum command-step validation;
- rate-aware joint speed and acceleration validation;
- optional workspace path validation (disabled by default for live streaming until the
  robot-specific table frame and tool geometry are calibrated);
- hardware connected/torque/fault checks;
- measured following-error and opposite-direction checks; and
- managed-backend motor effort/current checks.

The GUI smooths leader samples to fit the configured joint step, speed, and
acceleration limits before sending them to the guarded stream. Servo speed is no longer
additionally capped at the generic 250 ticks/s hardware setting, which had limited
follower motion below the host-side rate. The Teleoperation
tab reports how many samples were smoothed. Abrupt leader motion can therefore
make the follower lag briefly; the controller still rejects any command that
violates its limits.
GUI live teleoperation allows up to 1.2 rad/s joint speed and 6.0 rad/s²
acceleration; planned moves retain their configured limits. Gripper targets
ramp at 1.2 normalized units per second and stop 2.5% short of the
calibrated hard-close endpoint. When the follower stops moving while closing
toward the leader target, teleoperation eases open 0.5% and holds that opening
while the arm stream continues. Opening the leader gripper from the position where
contact was detected releases the latch. During the latch, the managed software effort threshold exempts
only gripper current/load; arm-joint effort checks remain active, and servo-reported
hardware faults still stop the stream.
When the measured starting pose sits just beyond a model limit, mapped targets are
clipped at that stream boundary so an outward leader twitch holds position instead of
raising a joint-limit error. Motion back into the legal range remains available.

The GUI records detailed JSONL diagnostics by default. Setup shows the session
file path and has a checkbox to turn per-sample records off. Each `teleop_frame`
contains the leader, desired, commanded, and measured five-joint positions;
per-joint following error; leader sample interval and age; and follower processing
time. This distinguishes irregular leader reads, host command pacing, and motor
tracking lag before changing rates or servo settings. File writes run on a
separate logging thread so serial command timing is not held up by disk flushes.
Every servo read also returns a packet error/status byte. A gripper position read can
therefore report an input-voltage fault even when no voltage register was requested.
The GUI now logs a leader gripper voltage sample about once per second during live
readout and captures a read-only voltage, configured limits, and status snapshot
immediately after a voltage fault. That snapshot may miss a brief voltage dip.

The selected stream frequency is used in the velocity/acceleration math. Lowering the
stream timer without changing this clock would be incorrect because the same angular
delta represents a lower physical velocity at 5 or 10 Hz than at 50 Hz.

## Backlog protection

Serial latency must not silently turn live teleoperation into delayed playback.

The GUI worker measures each leader sample's age before execution. A sample is
rejected and the follower is held if it is older than the larger of 150 ms or three
selected stream periods.

Follower processing time is also measured. If processing takes longer than the selected
period for three consecutive samples, teleoperation is terminated and the follower is
held. The Teach status line reports approximately:

```text
Live teleop 20 Hz — 25 samples; follower cycle 28 ms; queued age 3 ms.
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
7. joint-limit, command-step, speed/acceleration, following-error, and effort
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
