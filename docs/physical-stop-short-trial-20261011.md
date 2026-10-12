# SO-ARM101 short STOP physical evidence — 2026-10-11 20:32 EDT

**Repository:** `AgenticForge-Labs/soarm101-motion-sdk`  
**PR:** [#90](https://github.com/AgenticForge-Labs/soarm101-motion-sdk/pull/90), draft and unmerged  
**Implementation tested:** `ee7f9100ddb66e8ab85865e904b0683f0acf7eef`  
**Result:** **PASS for the script's software cancellation / stable encoder HOLD gates**, but **INCONCLUSIVE for actual moving-servo braking**. The host motion request was active, yet the pan servo's last pre-STOP `Moving` flag was false and its encoder was already unchanged before STOP. The previous large-motion STOP failure remains unresolved as a physical braking qualification.

## Evidence and immutable fingerprints

Operator-supplied ZIP: `short-stop-20261011-203217.zip`.

| File | SHA256 |
|---|---|
| ZIP archive | `2f043656425e62cccd4e8413bceadd360c9a06fceaaf17ac30ad915137b13c07` |
| `short-stop-20261011-203217.jsonl` | `cfc6a0d11f1fd3caf85cf6039693e9811dd705fac08f2fa7f04ac5f917ec4974` |
| `short-stop-20261011-203217.broker-events.jsonl` | `701a08481d1bbfd44fbea3b1a700117ecf874d75be64f5856bd39723e6426ca4` |
| `short-stop-20261011-203217.summary.json` | `e2c687d9694c5c1cfce972debee920b0610326453b6086c2755cbb18db25b824` |

Source files remain unchanged operator-owned evidence; do not repair/replace trace bytes to alter qualification. This source-controlled document records the independently calculated findings.

## Physical test setup and outcome

- Operator cleared the arm, explicitly enabled a calibration-bound lease, used measured Sleep (all five initial joint errors <=0.81°), requested one supervised `shoulder_pan +8°` move at **2°/s** and **10°/s²** with 20 Hz host setting. The human manually pressed ENTER while the SDK reported motion active.
- One SDK session: `3f12dc4d54e54471b627f53a781e0c7f`, **one** SDK device connection. Script reported `passed=true`, `stop_qualification_passed=true`; operator typed `STATIONARY`.
- Initial shoulder-pan encoder: **2044 ticks**, **-0.2637°**. STOP latch: **2127 ticks**, **+7.0330°**. Travel before STOP was **7.2967°**, or **91.2% of the commanded 8°**; desired final goal was +7.7363° and the stopped arm remained **0.7033° short**.
- Five-joint encoder feedback was unchanged over the post-latch stability samples at 0.25, 0.5, 1.0 and 1.5 s, the immediate post-STOP state, and another state 1 s later. Final trace feedback at +12.2696 s was also identical. This supports **no detectable encoder change for approximately 2.55 s after the latch** (encoder resolution is finite; 0.0° recorded is not a promise of mathematically zero physical vibration).
- `elbow_flex` changed by **+0.1758° (2 encoder ticks)** *during the commanded pan motion*, then remained unchanged through and after STOP. Other non-pan arm joints were unchanged at encoder resolution.
- Underlying raw write at trace **+9.717019 s** sent a **single** all-six-motor STOP/HOLD goal at the live encoder positions, `speed_raw=0`, `acceleration_raw=254`. The broker recorded STOP action start at **+9.714796 s**: approximately **2.22 ms** to the transport write. The last ordinary streamed command was at **+9.676964 s**, about **40.1 ms before** the STOP write. There were **no new raw position goals after the STOP latch**.
- The raw event record contains **86** position writes total: **84** ordinary position-stream writes (0/254), **1** initial all-motor latch (1/1) *before the moving trajectory*, and **1** corrected all-motor STOP latch (0/254). The initial 1/1 hold should not be mistaken for a second STOP write. `duplicate_raw_commands=0` in the tracer; the raw event counts independently confirm no repeated STOP latch.
- **Critical caveat: servo may already have been stationary before STOP.** The encoder first reached **2127 ticks** at trace **+9.589263 s**, and stayed 2127 at **+9.688587 s** before STOP began at **+9.714796 s**—about **125.5 ms without an observed step before STOP**. The latest sampled hardware `Moving` flag before STOP, at **+9.686727 s**, was **false** (as were its six immediately preceding samples). The host motion controller still reported an in-flight request, but this cannot be treated as proof that a physically moving joint was braked. The first post-write `Moving` sample at **+9.732984 s** was briefly **true**, despite encoder 2127; the terminal sample at **+11.255685 s** was **false**. The sampled `Moving` flag is not by itself a velocity measurement.
- The cancelled joint request returned **`ok=false`** with `joint motion cancelled`, not a late successful completion. STOP returned `ok=true`, `completed=true`, `holding=true` and `post_stop_stability.verified=true` with `max_drift_deg=0.0`. Agent authority was **revoked**; no operator recovery/rearm was requested.
- STOP HTTP action duration was **1.541 s**, deliberately including the **1.5 s stabilization monitor**. This is **not physical stopping latency**; direct write latency was ~2.2 ms from traced broker STOP start, while true electromechanical deceleration can only be bounded by sampled encoder feedback, not certified from this trace.
- All sampled hardware states were connected, nonfaulted and torque-enabled; the post-settle final state reported not moving. A brief `Moving=true` readback appeared immediately after the STOP write, without any measurable encoder step. This test did **not** certify active braking. Recorded effort showed at most pan load 56 raw, elbow load 40 raw, and current 1 raw during the trace; not a high-load validation.

## Comparison to original physical failure

The original operator-supplied `stop-debug-20261011-113611` trial (see `docs/feetech-stop-protocol-investigation.md`) showed **three STOP latches** with `speed_raw=1,acceleration_raw=1`, then **18.4° powered elbow motion** within ~1 s after acknowledged STOP, followed later by reversal. The new short trace shows **one** STOP latch at `0/254`, no subsequent goal, and no measurable arm-joint drift over ~2.55 s after the latch. These changes are consistent with the software fix, but the tests differ in initial pose, moving joint, mechanical loading and proximity to the target; they do **not** independently isolate speed/accel semantics or guarantee safety at other velocities and loads.

## Qualification limits / follow-up gate

1. **The present test does not demonstrate braking of a physically moving joint**: host-side motion remained active but the last pre-STOP pan encoder readings were unchanged and the servo `Moving` flag was already false. Passing one near-endpoint 2°/s cancellation does not qualify mid-trajectory STOP, the larger Overhead motion, elbow under load, all joint directions, fault-response dynamics or emergency stopping.
2. Do **not** close the original physical incident or release production native broker motion based on `passed=true` alone. Keep #90 draft pending operator-approved broader qualification, explicitly scoped measurement criteria and review of any physical traces.
3. Require code-reviewed and human-approved test plans before further moving hardware tests. Keep power cutoff accessible, reject drift/fault/rearm attempts and never automate a new START/STOP from agents or CI.
4. Preserve post-STOP stationary feedback checking; assess command-to-readback timing and ensure physical braking limits can be established independently of broker HTTP completion.
5. Production broker stays CLI-backed; no dependency on OpenShell, RoboConduit or sibling repos was added by this analysis.

**Integration status at time of analysis:** PR #90 open/unmerged; this was an attended local hardware test only, not a merge to `main`.
