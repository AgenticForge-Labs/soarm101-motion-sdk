# Feetech STS3215 STOP protocol investigation — 2026-10-11

**Status: source-confirmed command bug; candidate software correction only; physical STOP unqualified.**
**Source of truth:** Feetech hardware backend and shared SDK motion controller, not an agent-specific handler or edited trace.

## Source evidence

Official Feetech Python `sms_sts.SyncWritePosEx` writes seven bytes beginning at
STS `Acceleration` register 41:
```
[ACC, GoalPositionLow, GoalPositionHigh, 0, 0,
 GoalSpeedLow, GoalSpeedHigh]
```
This is a **new position-control goal**, not a special brake or HOLD opcode.
The two zero bytes are Goal Time (registers 44–45); they are not a corrupted
position or torque register. Each `groupSyncWrite.txPacket()` confirms the
bus transmission result, **not** each motor's acceptance or stationary state.

References (read 2026-10-11):
- Feetech's official Python SDK:
  https://github.com/ftservo/FTServo_Python/blob/main/scservo_sdk/sms_sts.py
- Waveshare ST3215 product/controller documentation, which explicitly says
  lower ACC means lower starting/stopping acceleration:
  https://www.waveshare.com/wiki/ST3215_Servo
- ST3215 controller register model/unit cross-check: ACC ~100 steps/s² per
  value, speed encoder steps/s, 0 speed meaning unrestricted:
  https://pkg.go.dev/github.com/hipsterbrown/feetech-servo/feetech
- LeRobot's existing SO-series setup and controller registers:
  https://github.com/huggingface/lerobot/blob/main/src/lerobot/motors/feetech/feetech.py
  and https://github.com/huggingface/lerobot/blob/main/src/lerobot/motors/feetech/tables.py

The device-specific permitted acceleration ceiling must be read/verified
before a new physical qualification: Waveshare's example describes a
different cap from the Feetech register's full byte range; the existing
SO-ARM101 streamed path already used 254, but that is **not evidence that
254 is a safe emergency braking profile** for every connected motor.

## Operator-supplied original trial

2026-10-11 `stop-debug-20261011-113611.zip`
SHA256: `90a70175cc3e52ba4936e128e8f9fe7bb9f9de7361eb12c43114e8fcdf30fcaa`.
Original motor trace SHA256:
`31b5b32687943c124fb35583ed36b555dcb68118191da2e191fdde2c5eb0a907`.
The original motor/event/summary files are unchanged evidence.

The last host-streamed Overhead command at trace +62.8961 seconds requested
elbow target 2969 ticks using `speed_raw=0`, `acceleration_raw=254`.
At +62.8986 the independent broker received STOP; it then emitted THREE
separate SyncWritePosEx HOLD batches at +62.9073, +62.9100 and +62.9134.
They requested elbow raw goal 2990 ticks with **speed 1 / acceleration 1**
(and additional motor current-position targets). Motion was still reported
by the hardware at +62.9200, but broker responded success at +62.922 s.
Elbow feedback was 2989 ticks (~82.8°) at STOP acknowledgement and
2780 ticks (~64.4°) at +63.934 s, ~209 ticks / 18.4° farther toward
Overhead. Shoulder also continued moving. By +92.131 s, before the
human-authorized recovery, elbow was ~2994 ticks again, close to the STOP
goal of 2990. The user explicitly reports **powered motor excursion and
reversal**, not passive gravity or momentum.

The recorded raw commands include NO new host-streamed motion command
during the ~1-second initial post-STOP excursion or subsequent return.
There is sparse encoder feedback between +63.9 and +92.1; the exact reversal
time, velocity peak, inner-loop actuator state and per-motor firmware
acceptance are unknown. The original `passed=true` qualification was false.

## Root cause assessment

**Confirmed SDK defect:** `FeetechBackend.stop()` issued a fresh
position-control command while simultaneously changing the inner servo
tracking parameters from the moving path's `0/254` to `1/1`.
The `1/1` combination imposed a very low goal-speed limit and a very
low acceleration/deceleration ramp while the servo was already following
a host-streamed path. Such a command is not an immediate physical brake;
it can prolong an internally controlled approach/reversal toward the
latched position. This is a strong mechanism consistent with the user
and trace evidence, though the exact firmware transient requires a
supervised hardware test to confirm.

**Confirmed second code issue:** A single `SOARM101.stop()` can reach
`FeetechBackend.stop()` through multiple independent paths:
- `MotionController.stop()` directly invokes backend STOP;
- the cancelled `MotionController._execute_plan()` error handler
  performs another backend STOP;
- `SO101Gripper.stop()` always invokes backend STOP, even if no gripper
  motion was commanded by this joint-only leg.

The trace's three HOLD writes over ~6 ms are consistent with those paths.
The ordering of the exact caller for each cannot be proven from existing
unlabeled raw-command events. Reissuing position-control goals as the
encoder changes can reset the servo's internal profile again; this is
another issue to resolve before live qualification.

**Not supported by the trace:** agent/model issuing new motion after STOP,
a changed saved Overhead target after STOP, Goal Time bytes being accidental
nonzero velocity, or passive drift as the primary operator observation.

## Candidate correction in open PR #90 (NOT on main)

The staged backend changes the position latch to the already used
`TELEOP_SERVO_SPEED_RAW=0` and `TELEOP_SERVO_ACCELERATION_RAW=254`
profile rather than `1/1`, keeping the held *measured encoder target*.
Hardware-free Feetech packet regression tests verify speed, acceleration,
and present-position goal. The broker's independent 1.5-second encoder
stability gate, fault latching and authority revocation remain in effect.

This is only a **source-supported candidate correction**, not a certified
braking algorithm: 0 allows maximum internal servo speed, and the
manufacturer-reseller information about maximum supported acceleration
varies. Neither code review nor simulated tests can demonstrate safe
deceleration, physical stop distance, or holding under actual load.
The duplicate software STOP paths have now been narrowed in this
draft branch: controller-owned cancellation suppresses the worker's
fallback write, and SOARM101.stop() does not call idle gripper STOP after
a successful arm HOLD. An independently cancelled worker still issues
its own fallback. These paths are covered by hardware-free tests, but
their combined physical timing remains unqualified.

## Resolution gates

1. Keep #90 in draft and normal production broker CLI-backed.
2. Verify the branch's new source-level STOP deduplication in full
   software tests and, later, guarded hardware traces: controller-issued
   STOP owns the cancellation fallback; idle-gripper STOP does not send
   another whole-arm latch. Exceptional direct handle cancellation
   retains its fallback hold. Faulted SDK cleanup closes transport
   without silently issuing new actuator goals. Any incomplete HOLD
   is still a failure. A true hardware single-latch claim remains
   unverified until an attended, controlled test.
3. Read the actual motor model/firmware, Operating_Mode, Goal_Position,
   Goal_Velocity, Acceleration, Maximum_Acceleration, Present_Velocity,
   Present_Load, Present_Current, Status and Moving (read-only, no torque
   or motion writes). Capture complete per-motor read results and packet
   errors. Current state cannot retrospectively prove prior firmware
   register contents, but exposes motor-specific supported limits.
4. Agree on a separate mechanically supported, unloaded, human-operated
   low-energy test protocol with an accessible physical power cutoff.
   Only after reviewing the physical limits, verify one controlled
   interrupted move with synchronized pre/post STOP encoder velocity,
   per-motor Goal Position/Speed/Acceleration readbacks, latency,
   motion cessation and stable HOLD over time.
5. Treat any active movement or re-acceleration after acknowledged STOP
   as test failure. **Do not rerun the prior large-angle Overhead trial
   to discover braking behavior.** No agent-driven motion, automatic
   rearm, retry, or production cutover until this gate passes.

Other architecture work (#91 onward) remains dependent on merging #90.
