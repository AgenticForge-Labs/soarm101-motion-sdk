# Leader → follower teleoperation

Live teleoperation is implemented through the same guarded motion stack used by the rest
of the SDK. Its host-side target limiter remains active while streamed Feetech position
commands use the same unrestricted servo speed and maximum acceleration settings as
LeRobot.

## Streaming design

The normal planned-trajectory command clock remains **50 Hz**. Live teleoperation has a
separate explicit stream rate because the current Feetech safety path performs synchronous
serial feedback work for every accepted follower sample.

The GUI keeps stream cadence and tracking response separate. Stream-rate choices are:

- **5 Hz — slow check**
- **10 Hz**
- **20 Hz — default**
- **50 Hz — experimental**

Tracking-response choices shape how quickly the follower closes the leader gap:

- **Slow · gentle** — 0.6 rad/s, 3.0 rad/s²;
- **Medium · current** — 1.2 rad/s, 6.0 rad/s²; this preserves the pre-preset behavior and is the default;
- **Fast · wrist-aware** — all five joints may use 100 deg/s; non-`wrist_flex` joints may use 1000 deg/s² while `wrist_flex` is capped at 500 deg/s².

The selected response limits are enforced twice: the GUI target limiter shapes the follower command toward the leader, and the same per-joint speed/acceleration ceilings are passed into the deterministic MotionController stream guard. Those session ceilings are clipped by the configured absolute stream envelope. Selecting a preset can therefore narrow response but never widen joint limits, command-step limits, following-error guards, effort/fault handling, or stale-sample policy. The Fast wrist-flex acceleration cap comes from a 2026-10-06 physical run where the former 1000 deg/s² response commanded a reversal while the wrist was still braking and the strict direction guard observed +0.057 rad of carry-through.

After both arms are connected, **Align follower and start** reads a fresh leader
pose, latches the follower's current positions before enabling torque, and moves
the follower's five pose joints to those leader angles with a guarded planned
move. The five joints start together and follow one timed trajectory. With
gripper mirroring selected, an opening gripper starts alongside the joint move;
its speed is chosen to approach the leader's measured opening near the end of
joint alignment. The GUI verifies arrival before streaming and completes any
remaining opening then. Any closing difference is handled by the live contact
guard after joint alignment. Keep the leader still
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

## Leader parking and pose handoff

The leader now has an explicit state independent of follower control:

- **FREE** means leader torque is off and the arm is back-drivable for hand teaching.
- **PARKED** means the SDK latched the leader's freshly measured position and enabled
  torque so it holds that pose.
- Starting aligned teleoperation or **Relink here — no motion** releases a parked leader
  before live readout begins.
- Disconnect also disables torque through the normal backend disconnect lifecycle.

Manual and Teleoperation both expose the same coordination controls. **Move leader →
follower pose** captures the follower fresh and performs a guarded joint move on the
leader. **Move follower → leader pose** does the reverse. Gripper matching is optional.
These are destination-arm moves: the source arm is not commanded.

**Relink here — no motion** is for intentional divergence. It latches the follower at its
own current position if needed, uses the current leader and follower configurations as the
new relative reference, and starts relative teleoperation without first moving either arm
into the other's pose. **Stop → Manual + park leader** supports a coarse-to-fine workflow:
use the leader to approach the task, stop following, park the leader, then refine the
follower with Manual angular/Cartesian controls.

The SDK default is 20 Hz. Rates above 20 Hz require an explicit GUI confirmation on
hardware and are not considered physically validated. A supervised 20 Hz Medium-response
session completed successfully on 2026-10-06; the full timing/STOP validation ladder below
still remains the promotion gate for treating that setup as validated.

Each streamed follower sample still performs the normal safety work:

- calibrated/model joint-limit validation;
- maximum command-step validation;
- rate-aware joint speed and acceleration validation;
- optional workspace path validation (disabled by default for live streaming until the
  robot-specific table frame and tool geometry are calibrated);
- hardware connected/torque/fault checks;
- measured following-error and opposite-direction checks; and
- managed-backend motor effort/current checks.

The GUI smooths leader samples to fit the selected Tracking-response speed and
acceleration limits plus the configured command-step limit before sending them to the guarded stream. Servo speed is no longer
additionally capped at the generic 250 ticks/s hardware setting, which had limited
follower motion below the host-side rate. Cartesian `move_linear()` now follows the
same principle: its host-side 50 Hz trajectory owns speed/acceleration shaping, while
the servo receives the responsive 0/254 profile instead of a second slower trajectory. The Teleoperation
tab reports how many samples were smoothed. Abrupt leader motion can therefore
make the follower lag briefly; the controller still rejects any command that
violates its limits.
Medium live teleoperation preserves the previous 1.2 rad/s joint-speed and 6.0 rad/s²
acceleration behavior. Slow halves those response limits. Fast uses the full configured
joint speed ceiling and full acceleration ceiling on the four non-wrist-flex joints, while
`wrist_flex` uses a 500 deg/s² acceleration ceiling. In every case the stream controller
independently enforces those per-joint session limits under the absolute configured envelope.
Planned moves retain their own requested/configured limits.

Live-stream direction monitoring remains fail-closed but is reversal-aware. A material
measured move opposite the command still stops the stream immediately unless the command
has just reversed direction. After a genuine reversal, the stream may accept at most 100 ms
of residual motion in the previous physical direction, and only while that carry-through is
non-growing within 0.5° of encoder/noise tolerance. Following-error, hardware-fault, effort,
joint-limit, and command timing checks remain active throughout the grace interval. Planned
joint trajectories do not use this exception and keep the strict unexpected-direction rule.

Gripper targets
ramp at 1.2 normalized units per second and stop 2.5% short of the
calibrated hard-close endpoint. When the follower stops moving while closing
toward the leader target, teleoperation eases open 0.5% and holds that opening
while the arm stream continues. Opening the leader gripper from the position where
contact was detected releases the latch. During the latch, the managed software effort threshold exempts
only gripper current/load; arm-joint effort checks remain active, and servo-reported
hardware faults still stop the stream.
The GUI shares one gripper motor speed preset across Manual, Teleoperation, Edit
recordings, and Run: Slow uses the original 250 raw ticks/s setting, Normal (the
default) uses 500, and Fast uses 1250. The selected speed is propagated to manual
moves, teleoperation alignment and live mirroring, Home/Rest gripper completion,
sequence gripper actions, and recorded-trajectory replay. Opening during teleoperation
alignment may run slower when necessary to coordinate with joint arrival; closing during
alignment still waits for the live contact guard. These faster settings have simulator
coverage but have not yet been verified on a physical arm.
If the follower loses only the status reply to a torque-enable write at
teleoperation start, the GUI turns torque off and retries up to twice with a
fresh measured-position latch. Other communication and hardware faults stop
the start attempt.
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

Follower processing time is also measured and reported, but it is diagnostic rather than
a stop condition by itself. An individual cycle can exceed the nominal period because of
serial jitter without creating delayed playback. Teleoperation stops and holds when the
actual queued leader sample age crosses the stale-data limit, or when another motion,
communication, hardware-fault, following-error, or effort guard trips. The Teleoperation
status line reports approximately:

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

Use 10 Hz as a diagnostic fallback when investigating transport or timing problems. It may
look visibly stepped during hand-following because commands are only updated every 100 ms.

### 3. 20 Hz

This is the practical default for hand-following. The period is 50 ms. Occasional follower
cycles longer than 50 ms are recorded as timing overruns, but they are not themselves a
reason to stop if queued sample age remains low. Growing queue age, a stale-sample stop,
communication error, or noticeably delayed STOP response still means the transport path
needs attention.

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
