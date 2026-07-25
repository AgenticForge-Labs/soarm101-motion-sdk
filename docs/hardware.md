# Hardware backend

The default backend talks directly to six Feetech STS3215 servos through `ftservo-python-sdk` at 1 Mbps.

| Function | Motor IDs |
|---|---|
| shoulder pan through wrist roll | 1–5 |
| `SO101Gripper` actuator | 6 |

Connection performs a ping/model check, reads EEPROM calibration, optionally compares it with stored calibration, and applies the SO-101 position-mode/PID and stock-gripper protection settings. It never enables torque unless requested.

Calibration search order:

1. Explicit `calibration_path`
2. `~/.config/soarm101/calibration/<robot_id>.json`
3. LeRobot calibration directories under `HF_LEROBOT_CALIBRATION` or `HF_LEROBOT_HOME`

The SDK reads and writes LeRobot's fields: `id`, `drive_mode`, `homing_offset`, `range_min`, and `range_max`. The LeRobot key `gripper` maps to SDK tool actuator `so101_gripper`.

## First physical smoke test

```bash
soarm101 ports
soarm101 diagnose --port PORT --allow-uncalibrated
soarm101 read --port PORT --robot-id ID
soarm101 move-joints --port PORT --robot-id ID --degrees 0 -5 5 0 0 --yes
```

Use no payload and remain near the centered pose. Confirm every joint direction before larger movements.
