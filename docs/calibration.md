# Calibration

SO-ARM101 calibration is based on the printed arm's repeatable mechanical end stops. The user does **not** estimate or manually place any joint at its midpoint.

## Live sweep procedure

Run:

```bash
soarm101 calibrate --port /dev/ttyACM0 --robot-id forge-arm
```

Torque is disabled for the entire sweep. When recording begins, move **every joint and the gripper repeatedly through its complete safe travel**, gently reaching both printed stops several times. Do not hold or force a joint against a stop.

The SDK samples all six encoders continuously. Raw encoder positions are unwrapped as the arm moves, so a joint may cross the `4095 -> 0` encoder seam without corrupting its measured range.

For each motor the SDK then:

1. Finds the minimum and maximum physical encoder positions observed during the live sweep.
2. Rejects a motor that was not moved far enough to represent a real stop-to-stop sweep.
3. Rejects travel of one full encoder revolution or more, because the stock printed SO-ARM101 is mechanically bounded.
4. Defines the physical joint zero as exactly halfway between the two observed extrema.
5. Computes the shortest valid Feetech `Homing_Offset` that maps that midpoint to the encoder half-turn reference.
6. Writes symmetric minimum and maximum position limits around that zero.
7. Reads the EEPROM values back and verifies that they match.

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

Collect several well-spread poses before trusting larger Cartesian moves. These measurements are the basis for a future optional geometric/TCP calibration layer; they are intentionally separate from encoder calibration.
