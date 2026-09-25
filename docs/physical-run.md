# First physical run — bench card

This is the short, ordered checklist for the first real SO-ARM101 follower session.
`TESTING.md` remains the complete validation record.

**Stop after any failed gate.** Do not compensate for unexpected behavior by raising
limits, disabling safety checks, or continuing to a later stage.

## Before connecting

- Secure the follower base.
- Remove payloads and anything from the gripper.
- Clear the complete arm workspace.
- Inspect servo daisy-chain cables and controller/power wiring.
- Confirm the power supply and servo voltage variant match.
- Keep the physical power switch or plug immediately reachable.
- Start with only the follower connected.

## Gate 1 — read-only discovery and diagnostics

Find the follower port with the GUI **Find Arms** feature or `soarm101 ports`.

Run diagnostics with torque off:

```bash
soarm101 diagnose --port PORT --robot-id ROBOT
```

Continue only if:

- all six expected STS3215 motors are present;
- model/status/communication checks are clean;
- voltage is plausible for the follower hardware;
- temperatures/current do not look abnormal at rest; and
- moving the relaxed arm by hand produces sensible encoder/readout changes.

## Gate 2 — native mechanical-stop calibration

For the first hardware validation, prefer this SDK's native mechanical-stop calibration
rather than importing an old external calibration.

In the GUI:

1. Open **Setup → Calibrate arm — mechanical stops** and click **Find Arms**.
2. Select **Follower**, confirm the discovered port/robot ID, and click **Connect for calibration (torque off)**. Do not press Enable.
3. Click **Start calibration sweep**.
4. Move all five pose joints and the gripper from one printed stop to the other and back until every gauge reaches **2/2** and **DONE**.
5. Do not force or hold an actuator against a stop.
6. Each pose joint must span at least 2048 ticks; the gripper must span at least 900 ticks. The displayed gauge scale may use that arm's prior calibration and is not the pass threshold.
7. Recording stops automatically when all six reach 2/2, otherwise the selected time limit applies. Wait for **CALIBRATION SAVED** before disconnecting.

Record:

```text
robot_id:
calibration_id:
shoulder_pan span:
shoulder_lift span:
elbow_flex span:
wrist_flex span:
wrist_roll span:
gripper span:
```

Verify both exist:

```text
~/.config/soarm101/calibration/<robot_id>.json
~/.config/soarm101/calibration/history/<robot_id>/<fingerprint>.json
```

Disconnect the calibration session and reconnect the follower normally before powered testing.

The backend independently refuses torque enable if a selected motor still has the
factory 0..4095 calibration range.

## Gate 3 — torque latch only

Place the relaxed arm near the mechanical midpoint, well away from stops.

Enable torque without commanding motion.

Continue only if:

- there is no jump on enable;
- the arm holds essentially where it was;
- no motor fault or effort trip appears; and
- STOP/HOLD and physical power remain immediately available.

Torque enable checks each current raw position against the active EEPROM Min/Max limits
before any Goal_Position or Torque_Enable write. A position up to 8 ticks (about 0.7°)
outside a limit is clamped inward to the limit and clearly logged; larger violations
still block enable. Confirm any inward correction is small and expected before continuing.

Relax again before moving to Gate 4.

## Gate 4 — five independent 2° joint tests

Use the CLI smoke test. Start with shoulder pan:

```bash
soarm101 smoke-test --port PORT --robot-id ROBOT --joint shoulder_pan
```

Default behavior is deliberately small: 2°, 0.05 rad/s, 0.20 rad/s², return to the
starting position, then relax.

Only after that joint passes, repeat for:

```text
shoulder_lift
elbow_flex
wrist_flex
wrist_roll
```

For every joint verify:

- initial torque latch causes no jump;
- the commanded positive direction agrees with the GUI/model convention;
- motion is smooth and small;
- it returns cleanly;
- there is no unexpected noise, binding, status fault, or effort trip.

A sign mismatch is a **stop condition for Cartesian testing** even if joint-position
feedback numerically follows the command.

## Gate 5 — small gripper test

Do not use **Open** or **Close** as the first powered gripper command.

With torque off, place the gripper roughly near mid travel. Then make small normalized
moves, for example:

```text
0.50 → 0.55 → 0.45 → 0.50
```

Verify physical direction, smoothness, current/load behavior, and return. Only later
approach the calibrated endpoints.

## Gate 6 — effort/current characterization

Open Setup → **Motor effort safety / characterization**.

With the already validated small motions:

- reset peaks;
- measure relaxed/idle readings;
- measure torque-on holding readings;
- repeat the small one-joint motions;
- record per-motor peak current and peak |load|;
- keep default trip thresholds initially.

If a normal unloaded motion trips, relax before changing a threshold. Do not disable the
guard merely to make a test pass.

## Gate 7 — fresh Home / Rest under this calibration

Only now save new Home and Rest poses. Their files will carry the active calibration ID.

Replay them at low speed. Home/Rest execution first completes the arm move and only then
commands the stored gripper position.

Do not reuse pre-fingerprint Home/Rest files for the first physical run.

## Stop point for the first foundational session

At this point it is reasonable to stop and review the recorded results.

Do **not** move on to Cartesian jogs, recorded trajectory replay, sequences, primitives,
or leader→follower teleoperation until:

- all five joint directions are confirmed;
- STOP/Relax behavior is confirmed;
- the gripper small-motion test passes;
- effort behavior has been characterized; and
- FK/TCP measurements are checked as described in `TESTING.md` and `docs/validation.md`.

Later stages then proceed in this order:

```text
measured FK/TCP
→ default 2 mm Cartesian jogs
→ leader calibration/readout
→ taught point replay
→ trajectory replay
→ sequences/primitives
→ relative teleop at 5 Hz
→ 10 Hz
→ validate the 20 Hz software default with timing evidence
→ experimental 50 Hz only after timing evidence
```

Calibration changes after any of those recordings invalidate their target binding.
Physical replay will fail closed until the affected artifact is deliberately reviewed
and re-saved/re-bound.
