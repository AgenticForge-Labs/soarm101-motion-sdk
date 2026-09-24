# Calibration

SO-ARM101 calibration is based on the printed arm's repeatable mechanical end stops. The user does **not** estimate or manually place any joint at its midpoint.

## Live sweep procedure

From the GUI, open **Setup → Calibrate arm — mechanical stops**, use **Find Arms**, select **Follower** or **Leader**, and click **Connect for calibration (torque off)**. From the CLI, run:

```bash
soarm101 calibrate --port /dev/ttyACM0 --robot-id forge-arm
```

The setup wizard uses the same calibration backend and a 90 second default time limit:

```bash
soarm101-setup --port /dev/ttyACM0 --robot-id forge-arm --recalibrate
```

Torque stays disabled for the entire sweep. Move **every joint and the gripper from one printed mechanical stop to the other and back**. Each actuator must complete two full end-to-end traversals; small reversals near a stop do not count. Do not hold or force an actuator against a stop.

The five pose joints must span at least **2048 encoder ticks (180°)**. The stock gripper uses a separate **900-tick** minimum. These are pass/fail thresholds, not the visual gauge endpoints. The GUI can scale a gauge from that arm's prior saved calibration so the display better reflects expected physical travel without changing the acceptance threshold.

The GUI shows traversal progress as 1/2 and 2/2. A gauge marked **DONE** means the two traversals were observed; it does not mean the calibration has been persisted yet. Recording ends automatically when all six actuators reach 2/2, otherwise the selected time limit applies. Wait for **CALIBRATION SAVED** before reconnecting for powered use. Cancel, a servo input-voltage fault, an incomplete sweep, or a save/verification failure preserves the previous saved calibration rather than silently replacing it.

The SDK samples all six encoders continuously. Raw encoder positions are unwrapped as the arm moves, so a joint may cross the `4095 -> 0` encoder seam without corrupting its measured range.

For each motor the SDK then:

1. Finds the minimum and maximum physical encoder positions observed during the live sweep.
2. Requires the actuator's minimum travel and two complete end-to-end traversals.
3. Rejects travel of one full encoder revolution or more, because the stock printed SO-ARM101 is mechanically bounded.
4. Defines the physical joint zero as exactly halfway between the two observed extrema.
5. Computes the shortest valid Feetech `Homing_Offset` that maps that midpoint to the encoder half-turn reference.
6. Writes symmetric minimum and maximum position limits around that zero.
7. Reads the EEPROM values back and verifies that they match.
8. Saves the current calibration alias and an immutable fingerprinted history copy.

Every saved calibration has a content-derived SHA-256 `calibration_id`. The current alias remains at `~/.config/soarm101/calibration/<robot_id>.json`, while immutable snapshots are stored under `~/.config/soarm101/calibration/history/<robot_id>/<calibration_id>.json`. Saved poses and other physical motion artifacts record their calibration provenance so replay can fail closed after a recalibration or cross-arm mismatch.

If calibration fails after EEPROM changes begin, the backend attempts to restore the exact calibration snapshot present before the sweep. If rollback also fails, do not enable torque until the motor EEPROM state has been inspected.

## Why midpoint comes from the stops

The previous procedure asked the user to place every motor approximately in the middle of its travel before the live range recording began. That made the zero point dependent on visual judgement. The printed SO-ARM101 already provides a more repeatable reference: its two physical stops. Using their midpoint gives the kinematic model a deterministic zero and keeps both limits symmetric around it.

## Kinematic validation after calibration

Motor calibration defines encoder zero and usable travel. It does not fit link lengths or TCP geometry. After calibration and the supervised one-joint smoke tests, compare predicted and physically measured TCP locations:

```bash
soarm101 kinematics-check \
  --port /dev/ttyACM0 \
  --robot-id forge-arm \
  --sample pose-01 \
  --x-mm 391.4 --y-mm 0 --z-mm 226.5 \
  --output kinematics-validation.jsonl
```

For a read-only repeated-point check that helps separate table/base offset from pose-dependent TCP or kinematic error, use `examples/tcp_table_calibration.py` with follower torque disabled.

Collect several well-spread poses before trusting larger Cartesian moves. These measurements are the basis for a future optional geometric/TCP calibration layer; they are intentionally separate from encoder calibration.
