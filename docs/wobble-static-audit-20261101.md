# SO-ARM101 low-speed wobble / Cartesian IK: read-only audit (2026-11-01)

Repository base: main 8eec5770e9670799b91ccd103966c4627d6907c4.
PR #79 was merged; independent draft PR #99 concerns SDK-owned MCP retirement.
This document and scripts/analyze_wobble_traces.py are diagnostic-only.
No production motion, limits, effort/fault, following-error, calibration, or STOP/HOLD behavior changes.

## 1. Time scaling and starvation

- motion/controller.py:287-293, 558-628: joint moves use one duration
  chosen from max(abs(target[j] - start[j])); all five pose joints share
  the same minimum-jerk normalized function:
  q_j(t) = q_j(0) + D_j * [10u^3 - 15u^4 + 6u^5], u=t/T.
- N = ceil(T * command_frequency_hz) + 1 command samples.
- The continuous per-cycle raw displacement at normalized time u is
  approximately abs(D_j) * 4095/(2pi) * 30*u^2*(1-u)^2 / (N-1) ticks.
  It approaches zero for every joint near the endpoint. A large-axis duration
  stretches small secondary-axis movement into many fractional-tick samples.
- Evaluate each real Sleep leg separately; joint-specific D and T cannot be
  inferred from the standard generic pose, because mechanical-stop calibration
  and the observed starting pose are authoritative.

## 2. Raw encoder conversion and write coalescing

- calibration.py:69-85: raw = int(round(radians*4095/(2pi) + center_raw)),
  with center_raw=(range_min + range_max)/2. Python round is ties-to-even;
  the final int does not truncate an unrounded float. Calibration bounds
  still reject out-of-range requests.
- motion/controller.py:1103-1128, 1550-1590: planned pose samples remain
  floats until stream-time encoder-key construction. A nonfinal sample is
  omitted only if the five-joint raw tuple equals the last SENT tuple.
  An unchanged minor joint may be included in another joint's write.
- hardware/feetech.py:570-657: each sent floating goal is converted again,
  preserving bounded inward-only recovery for near-endpoint measurements.
- This gives a testable quantized-hold/step mechanism, not proof that the
  mechanical wobble comes from write suppression. One-shot final_target
  rocked in previous supervised physical tests (TESTING.md:1380-1396).

## 3. Half-duplex servo bus / timing

- motion/controller.py:427-433: absolute deadline sleep, no compensation
  for overrun except a late-command fault check.
- motion/controller.py:1530-1605: normally 50 Hz, with full guard polling
  every ceil(trajectory_feedback_interval_s * frequency); the default
  interval is 0.10 s (config.py:102-104). A last-sample guard is also run.
- motion/controller.py:1207-1243: get_hardware_state(), then
  read_joint_positions() on guarded samples, followed by following-error
  and unexpected-direction checks. The live stream instead monitors
  every accepted sample (controller.py:1938-2040), normally 20 Hz.
- hardware/feetech.py:290-310, 515-568, 834-854:
  serial per-servo reads are used for positions, Moving and Status; a
  grouped SyncWritePosEx sends position, speed and acceleration. This
  is NOT a SyncRead batch every planned command tick.
- hardware/managed_feetech.py:510-546: effort monitoring can add
  Present_Current and Present_Load reads at guarded checks and fails closed.
- host config (config.py:143-151) suggests P=32, I=0, D=32 but actual
  EEPROM values may differ unless an explicit configure operation ran.
- SyncWritePosEx contains Goal_Position, speed and accel fields; only
  describing this as Goal_Position-only would be incorrect.
- Single-sample cycle jitter is not established by static code.

## 4. Different command profiles

- Live streaming and exact accepted teleop replay:
  controller.py:1855-2040 and examples/motion_quality_replay.py:356-373;
  normally 20 Hz, every accepted joint sample uses raw speed/accel 0/254,
  full monitor on every sample. Host caller also schedules recorded timing.
- Saved Trajectory.play_trajectory():
  controller.py:1139-1201, 1607-1684; retime/resample at configured
  command frequency (normally 50 Hz), write every resampled joint frame
  at 0/254, plus gripper writes/reads.
- Generated joint move_joints():
  controller.py:591-628, 1530-1605, 1694-1815; 50 Hz nominal,
  0/254 by default, whole-arm duplicate raw tuple suppression.
- Cartesian move_linear():
  controller.py:630-965, 1064-1101, 1816-1851; 50 Hz nominal;
  speed_raw varies PER JOINT PER SENT WAYPOINT via
  ceil(abs(raw_next-raw_previous)/(0.8*dt)), bounded 1..3400,
  acceleration_raw=254. Note raw_delta=0 -> speed 1.
- Joint final_target:
  controller.py:1371-1465 and 1694-1815; one endpoint SyncWrite,
  separately computed per-joint speed over plan duration,
  acceleration_raw=254. Uses guarded polling and workspace checks for
  asynchronous intermediate progress where requested.
- Differences in safety polling, gripper bus I/O, and write suppression
  prevent an interpretation that all paths share identical transport.

## 5. Cartesian IK and physical coordinate limitations

- model.py:57-83, 186-225: five revolute pose joints plus gripper.
  A five-DOF chain generally cannot independently satisfy arbitrary 6D
  end-effector pose; wrist/base coupling depends on actual geometry and TCP.
- ik.py:18-33, 66-88: exact, compatible, position_only and look_at are
  distinct orientation constraint modes. Default compatible is NOT a
  generic full 6D exact-pose constraint.
- ik.py:129-264: bounded scipy least_squares(method="trf"), with
  r=[100*position_error, orientation_residual, 0.02*(q-seed),
  0.002*(q-center)/span]. If soft regularization blocks feasible
  tolerances, a task-only refinement is attempted without weakening
  the acceptance threshold. No explicit LM/DLS Jacobian loop.
- controller.py:630-835: sequential seed from previous joint solution,
  multi-start retry, reverse-path fallback for position_only, 0.50 rad
  maximum jump and Cartesian-constrained smoothing. config.py:86-93:
  5 mm waypoint density, 0.5 mm solver position tolerance, although
  Cartesian samples are solved at the final host command rate.
- PLAN.md:720-770 documents physical/model scale mismatch and
  unexplained lateral drift. Correct internal FK/IK does not establish
  physical TCP geometric accuracy.

## Hardware observations already recorded (qualitative)

- TESTING.md:1125-1147: exact teleop-command replay was smoother than
  programmed motion; 20 vs 50 Hz did not materially change observed shake.
- TESTING.md:1380-1396: corrected final_target execution still rocked.
  Micro-waypoint writes alone therefore cannot explain the failure.
- PLAN.md:900-944: generated return to folded Sleep was particularly
  shaky; taught sleep2 was much smoother; faster 80/500 testing
  improved subjective smoothness; near-end deceleration remained suspect.
- TESTING.md:2419-2428: 1 mm IK knot spacing did not resolve the
  Cartesian shake; command-rate IK implementation also remained shaky.
- These reports are evidence about past supervised observations, NOT
  quantitative joint-tick statistics or proof of a single cause.

## Trace analysis: read-only

Inputs are locally retained JSONL from:
- examples/motion_quality_walkthrough.py (study and teleop-command replay)
- examples/motion_quality_trace.py (programmed 20/50 Hz and final_target)
- examples/sleep2_comparison.py (canonical Sleep / sleep2 conditions)
- GUI teleop session logs containing teleop_frame events
- PassiveBackendTrace, motion/trace.py:1-370

Example, using actual files saved by local experiments:

    python scripts/analyze_wobble_traces.py \
      --trace teleop=/actual/path/teleop-command-replay-poststudy.jsonl \
      --trace planned=/actual/path/program-trace-50hz.jsonl \
      --trace final_target=/actual/path/program-trace-final-target.jsonl \
      --trace sleep2=/actual/path/sleep2-comparison.jsonl \
      --out-json /tmp/soarm-wobble.json \
      --out-csv /tmp/soarm-wobble.csv

Use --segment Sleep to select marked Sleep segments in a single file.
Use --gripper-raw-range MIN MAX (and --gripper-reversed if drive_mode=1)
ONLY when copied from the matching real calibration. Otherwise the
sixth actuator's raw-tick metrics remain NA, not invented.

The output includes per-joint zero-increment run-length histogram, maximum
hold cycles, held-cycle fraction, final-25%-of-move step-impulse count/rate,
measured encoder sign reversals during monotonic commands, peak/RMS
following error, coarse velocity cross-correlation lag, estimated sub-tick
tail, quantized flat tail, and observed command-write gaps.

Important evidence limits:
- PassiveBackendTrace records only actual writes. It does NOT record
  unsent quantized or continuous planner samples. A skipped controller
  cycle is reconstructed from nominal timing; write-call timestamps are
  taken after the call returns.
- Sub-tick tail is an estimate based on differences between available
  float command goals divided by elapsed nominal cycles. It is NOT the
  exact unrecorded planned velocity, and is not meaningful for a one-shot
  target.
- Following error compares to the latest SENT command. Measurement
  feedback is sparse; time-resampling for correlation does not increase
  physical measurement resolution. Treat lag estimates conservatively.
- A gripper command may have only normalized 0..1 units, not raw encoder
  or angular values. Missing real calibration means NA for tick metrics.
- Guarded read/write timing, bus turnaround, individual servo firmware
  response and EEPROM P values still need a separate supervised,
  nonintrusive physical qualification. Never bypass guards to improve
  sample counts.

## Interpretation gate

If teleop replay stays smooth while planned joint motion shows longer
quantized holds/impulses at the same pose, pursue host trajectory shape
and minor-axis pacing in a separately reviewed change. If all forms
show measured reversals while goals remain monotonic, investigate servo
load/friction/backlash, gravity and inner position tracking. If lag or
jitter increases around safety polls, profile bus timing without
removing any checks. For Cartesian failures, first log target pose,
orientation mode, residual, Jacobian condition number, joint margins,
seed distance and physical/model coordinate agreement. No controller
repair is authorized by this document alone.
