# Direct control API

The SDK is designed for direct programmatic control of SO-ARM101 without ROS or LeRobot. `SOArmAPI` provides an xArm-inspired convenience layer while still using the native SO-ARM101 motion planner, IK, calibration, workspace checks, following-error monitoring, motor-effort interlocks, and cancellation behavior.

```python
from soarm101_motion import SOArmAPI

with SOArmAPI(port="/dev/ttyACM0", robot_id="forge-arm") as arm:
    arm.motion_enable(True)

    # Joint command. Degrees are convenient for interactive scripts.
    arm.set_servo_angle(
        [0, -20, 35, 0, 10],
        speed=20,
        mvacc=60,
        is_radian=False,
        wait=True,
    )

    # Absolute Cartesian target in base/world coordinates.
    arm.set_position(
        x=250,
        y=50,
        z=180,
        speed=20,
        wait=True,
        orientation_mode="position_only",
    )

    # Relative Cartesian move in the current tool/TCP frame.
    arm.set_tool_position(
        x=5,
        z=-2,
        speed=10,
        wait=True,
        orientation_mode="compatible",
    )

    # The gripper is a separate actuator: normalized position plus speed.
    arm.set_gripper_speed(180)
    arm.set_gripper_position(0.5, speed=160, acceleration=20)

    print(arm.get_servo_angle(is_radian=False))
    print(arm.get_position_values())
    print(arm.get_motor_effort("elbow_flex"))

    arm.move_gohome()
```

If the current/load safety interlock trips, the SDK holds the measured motor positions and aborts the active motion. After inspecting the arm and removing the obstruction, clear the latch explicitly:

```python
print(arm.effort_trip_message)
arm.clear_effort_trip()
```

See [effort-safety.md](effort-safety.md) for configuration and threshold semantics.

`SOArmAPI` intentionally does **not** claim xArm compatibility for controller features the hardware does not provide. The STS3215 current/load signals support useful contact and collision detection, but they are not a calibrated six-axis force/torque sensor. The software hold is not a certified emergency stop.

Guarded continuous joint streaming is available for leader/follower teleoperation and other low-latency control loops:

```python
arm.servo_j_start()
arm.servo_j([0.0, -0.30, 0.55, 0.0, 0.15], is_radian=True)
arm.servo_j([0.01, -0.29, 0.54, 0.0, 0.16], gripper=0.7)
arm.servo_j_stop()
```

Each streamed sample is checked against calibrated/model joint limits, maximum command step, joint speed and acceleration, the configured workspace envelope, following error, hardware faults, and motor-effort interlocks. Normal point-to-point and trajectory motion is excluded while a joint stream is active.

This streaming path is implemented and tested in simulation, but it is **not yet physically validated**. Before hardware teleoperation, complete the calibration/direction/kinematics gates and the dedicated low-speed streaming checks in `TESTING.md`. Cartesian velocity/servo streaming is still not implemented.
