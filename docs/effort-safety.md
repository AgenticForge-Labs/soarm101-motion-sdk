# Motor-effort safety interlock

The STS3215 servos report motor current and signed load feedback. The SO-ARM101 SDK uses those signals as a **software contact/collision guard** while torque is enabled.

This is not a force/torque sensor. Current and load correlate with mechanical effort, but they are affected by gearbox friction, linkage geometry, temperature, joint position, payload, and supply voltage. The SDK therefore reports and thresholds the native servo feedback rather than claiming calibrated force in newtons.

## Default behavior

`SOARM101Config` enables effort monitoring by default with initial hobby-arm guardrails:

```python
SOARM101Config(
    effort_safety_enabled=True,
    effort_current_trip_raw=250,
    effort_load_trip_raw=850,
    effort_trip_consecutive_samples=2,
)
```

These values are starting safety thresholds, not experimentally validated limits for every printed arm. Physical validation should record normal current/load during unloaded and expected-payload motions and tune the thresholds before unattended use.

A motor must exceed a threshold for the configured number of consecutive state samples before the interlock trips. Per-motor overrides allow sensitive joints or the gripper to use different limits:

```python
config = SOARM101Config(
    port="/dev/ttyACM0",
    robot_id="forge-arm",
    effort_current_trip_raw=250,
    effort_load_trip_raw=850,
    motor_current_trip_raw={"so101_gripper": 180},
    motor_load_trip_raw={"so101_gripper": 500},
)
```

Set either global threshold to `None` to disable that signal while retaining the other one.

## What happens on a trip

When any monitored motor repeatedly exceeds its current or load threshold, the backend:

1. records the motor, measured value, and configured threshold;
2. latches the effort-safety interlock;
3. commands a software hold at the currently measured motor positions;
4. reports the hardware state as faulted;
5. causes the active joint, Cartesian, or gripper motion to abort through the existing motion-safety path; and
6. blocks further powered motion until the trip is explicitly cleared.

The hold leaves torque enabled. It is a software safety stop, **not** a certified emergency stop and not a replacement for accessible physical power removal.

After removing the obstruction and inspecting the arm:

```python
arm.clear_effort_trip()
```

The next hardware-state sample immediately evaluates current/load again, so clearing the latch without removing the cause will re-trip it.

## Reading effort directly

With `SOArmAPI` on physical hardware:

```python
print(arm.get_motor_effort("elbow_flex"))
# {'current_raw': ..., 'load_raw': ...}
```

`load_raw` is decoded from the STS3215 sign-magnitude `Present_Load` register. Safety comparisons use its absolute magnitude. `current_raw` is the native `Present_Current` register value.

## Gripper contact behavior

Ordinary gripper commands remain covered by the same current/load interlock as the arm joints. A lower gripper-specific threshold can therefore stop a normal commanded close when the fingers contact an object.

Live leader→follower teleoperation also has a separate contact-hold behavior for gripper mirroring. The follower target is ramped rather than jumped toward the leader command and stops short of the calibrated hard-close endpoint. If the measured follower gripper stops progressing while it is closing, teleoperation eases it open slightly and latches that opening while the five arm joints continue streaming. Opening the leader gripper from the contact position releases the latch.

While this teleoperation contact latch is active, the managed software effort threshold is exempted **only for gripper current/load** so expected grasp contact does not terminate the whole arm stream. Arm-joint effort checks remain active, and servo-reported hardware faults still stop teleoperation. This behavior is a convenience contact heuristic, not calibrated grasp-force control.
