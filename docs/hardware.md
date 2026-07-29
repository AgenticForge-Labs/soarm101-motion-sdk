# Hardware backend

The hardware backend controls six Feetech STS3215 servos at 1 Mbps through the pinned `ftservo-python-sdk==2.0.0` package.

| Function | Motor IDs |
|---|---|
| shoulder pan through wrist roll | 1–5 |
| `SO101Gripper` actuator | 6 |

## Before power

Confirm the motor, controller-board, and power-supply voltage variants match. Secure the base, remove payloads, clear the workspace, inspect every daisy-chain cable, and keep physical power accessible.

## Simplest assembled-arm setup

When motor IDs 1–6 are already assigned:

```bash
soarm101-setup --robot-id forge-arm
```

The port is auto-selected when exactly one serial adapter is connected. The wizard:

1. Connects without enabling torque or rewriting configuration.
2. Verifies all six IDs, models, diagnostics, and status values.
3. Applies the recommended position/PID and gripper-protection settings.
4. Keeps a usable EEPROM calibration or guides a new center-and-sweep calibration.
5. Saves `~/.config/soarm101/calibration/forge-arm.json`.
6. Disconnects with torque off and prints the next smoke-test command.

Use `--recalibrate` to deliberately replace an existing EEPROM calibration.

## New loose motors

Fresh servos normally share a factory ID. Assign them one at a time before assembling the full daisy chain:

```bash
soarm101 setup-motors --port PORT
```

Connect exactly one motor at every prompt. The command first tries ID 1 at the common factory baud rates, so normal setup completes quickly instead of scanning thousands of combinations. For unusual settings, pass `--initial-id` and `--initial-baudrate`.

The setup operation disables torque, unlocks EEPROM, writes the target ID and 1 Mbps baud rate, verifies the result, and relocks EEPROM. It also attempts to relock EEPROM if a later verification step fails.

## Normal operation

Normal SDK connections, `diagnose`, and `read` do not enable torque or rewrite motor configuration.

Calibration search order:

1. Explicit `calibration_path`
2. `~/.config/soarm101/calibration/<robot_id>.json`
3. LeRobot calibration directories under `HF_LEROBOT_CALIBRATION` or `HF_LEROBOT_HOME`

## First powered test

```bash
soarm101 smoke-test --port PORT --robot-id forge-arm --joint shoulder_pan
```

Use no payload and test one joint at a time. Confirm the expected direction and a clean return before moving to the next joint.
