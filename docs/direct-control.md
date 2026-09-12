# Direct control API

The SDK is designed for direct programmatic control of SO-ARM101 without ROS or LeRobot. `SOArmAPI` provides an xArm-inspired convenience layer while still using the native SO-ARM101 motion planner, IK, calibration, workspace checks, following-error monitoring, and cancellation behavior.

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

    arm.set_gripper_position(0.5)
    print(arm.get_servo_angle(is_radian=False))
    print(arm.get_position_values())

    arm.move_gohome()
```

`SOArmAPI` intentionally does **not** claim xArm compatibility for controller features the hardware does not provide. There is no fabricated force/torque sensing, collision-sensitivity controller, industrial emergency stop, or hardware trajectory blending layer.

The next hardware-validation milestone is continuous/streaming control (`servo_j`-style joint streaming and Cartesian velocity/servo modes). Those should only be added after the stop-derived calibration, joint directions, kinematic zero, and guarded Cartesian paths have been verified on a physical arm.
