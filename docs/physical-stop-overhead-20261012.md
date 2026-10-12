# SO-ARM101 attended multi-joint Overhead STOP — 2026-10-12 11:32 EDT

**Repository:** `AgenticForge-Labs/soarm101-motion-sdk` — PR [#90](https://github.com/AgenticForge-Labs/soarm101-motion-sdk/pull/90), draft and unmerged.
**Operator trace:** `overhead-stop-20261012-113217` (measured physical `/dev/ttyACM1`).
**Result:** **PASS** for the attended, 6°/s, 30°/s², single saved-Overhead
interruption under the trial's physical-moving and stable-HOLD criteria.
This is *not* proof of hardware safety certification, arbitrary speeds,
loaded trajectories, workspace clearance or an emergency-stop function.

## Preserved operator evidence

These files were supplied by the human operator and were not modified.

| Local trace | SHA256 |
|---|---|
| `overhead-stop-20261012-113217.summary.json` | `3f72a72cab86eb597f65b92d0efe32358228f734ac69f07bdb5c64fb271ae488` |
| `overhead-stop-20261012-113217.jsonl` | `70c61ebf1aa99ea9e8cc7c05362415241cbfaf3d1bf5c14d3414d96489131eec` |
| `overhead-stop-20261012-113217.broker-events.jsonl` | `510c0e306eaf66970b2f1a5a5d6104144b3b103c7dba7517431e3edabe4e7abf` |

## Physical scenario and measured evidence

- One persistent SDK broker connection, one calibration-bound
  `agent_start_overhead` go-pose, joint-only, initiated by explicit
  `OVERHEAD_START` human confirmation from a measured stationary
  torque-held start; no preparatory Sleep, gripper close, automatic recovery
  or second movement. The saved Overhead target was checked against the
  already-calibrated executable joint limits. The command trajectory used
  20 Hz host streaming, 6°/s speed and 30°/s² acceleration.
- Planned change from starting posture: pan **−5.71°**, shoulder lift
  **+78.07°**, elbow flex **−89.93°**, wrist flex **+40.09°**,
  wrist roll **−0.18°**.
- The operator pressed ENTER while multiple motors were physically
  moving. The passive final 0.30-s snapshots show independently
  changing actual encoder positions on shoulder lift (1212→1230),
  elbow flex (2771→2750) and wrist flex (2803→2813).
  The Feetech `Moving` flag was also true across those snapshots.
- At STOP request, shoulder lift had progressed **30.07°** and had
  **48.00°** remaining; elbow flex had progressed **33.41°** and had
  **56.53°** remaining; wrist flex had progressed **15.30°** and had
  **24.79°** remaining. The script's multi-joint, physically in-flight
  evaluator independently marked all three joints `verified=true`.
- Trace marker `broker_action_start(stop)` at +17.927929 s.
  One all-motor raw HOLD position write followed at +17.930024 s,
  **2.10 ms** later, using `speed_raw=0, acceleration_raw=254`.
  The previous ordinary streamed raw position command was at +17.899119 s,
  about **30.9 ms** before the HOLD latch. **No new position goals were
  issued after the HOLD latch.**
- First post-STOP encoder observation at +17.958406 s, about **28.4 ms**
  after the raw HOLD write. Through its 1.5-s settling check the maximum
  measured transient drift was **0.7033°** on elbow flex; subsequently
  all five encoder readings were unchanged at their resolution, including
  another one-second state check. A STOP HTTP response duration of
  **1.553 s includes the intentional 1.5-s stability monitor** and is
  *not* the electromechanical stopping latency.
- STOP reported `accepted=true, completed=true, holding=true` and
  `post_stop_stability.verified=true`. The interrupted go-pose request
  returned `joint motion cancelled` rather than success. The human
  calibration-bound motion lease was revoked; summary recorded
  `stop_qualification_passed=true, passed=true`, with no recovery.
  Raw trace has 206 raw-command writes (205 streamed trajectory writes
  and exactly **one** STOP/HOLD write); `duplicate_raw_commands=0`.

## Safety and architectural interpretation

This directly improves on the earlier 2026-10-11 trial where an
Overhead-related STOP produced roughly 18.4° elbow movement after the
HTTP STOP acknowledgment. The corrected responsive 0/254 HOLD and
single-writer STOP were physically exercised here at modest speed with
moving shoulder, elbow and wrist. The record supports a genuinely
interrupted multi-joint motion, not a near-endpoint or stationary-encoder
false positive. It does not independently isolate every contribution of
the earlier fix, nor qualify higher energy, direction, external loads,
gripper motion, fault injection, or arbitrary equipment.

**Integration at evidence capture:** PR #90 remained draft/unmerged,
production native SDK broker motion remained disabled, and no
OpenShell/RoboConduit or other sibling system was activated. No agent
initiated the physical motion. The operator may separately choose to
return to Sleep via the existing direct trusted CLI; that is a NEW
human-commanded move, not a replay/resumption under revoked broker
authority.
