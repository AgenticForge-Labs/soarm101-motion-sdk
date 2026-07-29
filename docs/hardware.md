# Hardware backend

The default backend talks directly to six Feetech STS3215 servos through the pinned `ftservo-python-sdk==2.0.0` package at 1 Mbps.

| Function | Motor IDs |
|---|---|
| shoulder pan through wrist roll | 1–5 |
| `SO101Gripper` actuator | 6 |

## Before power

Confirm the motor voltage variant, controller-board voltage, and power-supply voltage match. Secure the base, remove payloads, clear the workspace, inspect the daisy-chain cables, and keep physical power accessible.

## New loose motors

Assign IDs and baud rate with only one motor connected at a time:

```bash
soarm101 setup-motors --port PORT
```

The command follows the same essential sequence as LeRobot's setup helper: find the isolated motor, disable torque, unlock EEPROM, assign the target ID, set 1 Mbps, verify the motor, and relock EEPROM.

## Read-only diagnosis and explicit configuration

```bash
soarm101 diagnose --port PORT --robot-id ID --allow-uncalibrated
soarm101 configure --port PORT --robot-id ID
```

`diagnose` does not enable torque or write configuration. `configure` is the explicit one-time operation that writes position mode, return delay, acceleration/PID values, phase behavior, and stock-gripper protection settings. Normal connections default to `configure_motors_on_connect=False`.

Connection still pings all six IDs, verifies model numbers, reads EEPROM calibration, and optionally compares it with a stored calibration file.

Calibration search order:

1. Explicit `calibration_path`
2. `~/.config/soarm101/calibration/<robot_id>.json`
3. LeRobot calibration directories under `HF_LEROBOT_CALIBRATION` or `HF_LEROBOT_HOME`

## First physical smoke test

```bash
soarm101 ports
soarm101 diagnose --port PORT --robot-id ID --allow-uncalibrated
soarm101 configure --port PORT --robot-id ID
soarm101 calibrate --port PORT --robot-id ID --seconds 30
soarm101 read --port PORT --robot-id ID
soarm101 smoke-test --port PORT --robot-id ID --joint shoulder_pan
```

Use no payload and test one joint at a time. Confirm direction and a clean return before testing the next joint.
