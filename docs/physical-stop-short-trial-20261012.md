# SO-ARM101 attended moving-pan STOP evidence — 2026-10-12 08:23 EDT

**PR:** #90 (`feature/broker-independent-stop-faults`), draft and unmerged.
**Run:** `short-stop-20261012-082341` on `/dev/ttyACM1`.
**Original script result:** `passed=false` (pre-STOP evidence evaluator rejected despite
verified stable STOP); **physical interpretation:** strong observed low-speed
moving-pan STOP evidence with no meaningful post-STOP encoder displacement.
**User observation:** shoulder pan visibly stopped immediately.
**Scope:** unloaded, single-joint, 2°/s, 10°/s², +8° target. Not higher-energy
Overhead or emergency-stop qualification.

## Operator-supplied original evidence (unaltered)

| File | SHA256 |
|---|---|
| `short-stop-20261012-082341.summary.json` | `134c13da569e7bfab56883a11a20e1b2d72359da40fef9fd03d6e6b13071b84d` |
| `short-stop-20261012-082341.jsonl` | `f8f7ec84196f74814984bc0600e42d53a7673be131a31bd71f925168342bd823` |
| `short-stop-20261012-082341.broker-events.jsonl` | `6591b5244d6f5a9d9e1aec148bc8b68a569715b31d5b944b6fca3b9fa546726e` |

The original summary remains an immutable record of the previous evaluator.
Do **not** rewrite it to set `passed=true`. Its rejected result was traced to
qualifier implementation, not missing STOP/cancellation evidence.

## Timeline (passive trace monotonic seconds)

- +11.020: human-authorized joint-only Sleep preparation via the same trusted
  broker. +17.778: Sleep verified at <1.1° worst joint error, gripper excluded.
- +21.512: pan +8° joint motion starts. The encoder shows ongoing physical
  motion. In the last ~0.30 s, feedback changed **2091 -> 2098 ticks**
  (0.6154°); intermediate samples were 2095 and 2096. Median pace
  approximates the requested 2°/s; the latest reading was at +25.6486.
- +25.67165: operator ENTER triggers STOP, at **4.3077°** movement progress
  and **3.6923°** to target. Last changed encoder sample was ~0.023 s old.
  Yet the last four sampled Feetech `Moving` flags were all **false**,
  directly contradicting encoder-position progression.
- +25.6725: broker STOP starts; +25.6750: one raw position latch at live
  shoulder-pan **2098** ticks, `speed_raw=0,acceleration_raw=254`, ~2.45 ms
  after broker STOP action marker. Last host-streamed raw command was at
  +25.6372; no new position target was written after STOP.
- +25.6943: first post-write encoder still 2098. +25.9472: 2097 ticks
  (0.0879° backward settling), then 2097 through the rest of the 1.5-s
  stabilization monitor and the subsequent one-second readback.
- The interrupted joint POST returned `ok=false`,
  `joint motion cancelled`; STOP returned `ok=true`, `holding=true`,
  `post_stop_stability.verified=true` with 0.0879° max drift.
  Human lease was revoked and no further movement was requested.
- STOP HTTP response took 1.546 s including the mandatory 1.5-s
  stabilization monitor. This is **not a measured braking latency**.
  The last pre-STOP and first post-STOP encoder samples were ~46 ms apart;
  first post-latch feedback arrived ~19 ms after the raw write. True
  stopping time is not resolved more precisely by this sampling cadence.

The trace reports 138 raw transport position writes: 85 during preparation,
52 during the pan move, **one during STOP**. Its
`duplicate_raw_commands=1` counter does not represent a second STOP latch;
the recorded STOP phase contains exactly one raw goal.

## Verified evaluator defect

1. The 10-Hz encoder transition occurred at the newest changed sample
   (+25.6486). Old code recorded time from the **prior** distinct sample
   (+25.5484), converting ~0.023 s freshness into ~0.123 s and failing the
   0.12-s cutoff. New code timestamps the *transition's new sample*.
2. The Feetech `Moving` bit was false across multiple samples even as
   the encoder physically changed. New logic retains its freshness and
   contradiction as diagnostics, but qualifies moving evidence from fresh
   directional encoder change (>=0.15° in the recent window) and >=2°
   remaining, not the binary motor status bit alone.
3. Frozen real-sample regression asserts the exact trace passes the evidence
   subroutine under the corrected logic. A stationary encoder despite
   `Moving=true` must fail.

## Safety boundary / next milestone

This strengthens the evidence that the corrected one-latch 0/254 STOP
worked for **this** slow single-pan movement; it does not prove actuator
braking at arbitrary speed, load, direction or with multiple moving axes.
It does not retrospectively erase the prior real 18.4° elbow motion after
STOP. Keep PR #90 unmerged pending a separately reviewed human-attended
broader physical qualification (including whichever axes in Overhead are
necessary). Do not silently make the legacy `--trial overhead` default;
it involves multiple motion legs and the previous failure mode. Preserve
calibration, joint limits, fault checks, route clearance and physical cutoff.
