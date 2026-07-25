from soarm101_motion import SOARM101

with SOARM101(port="/dev/ttyACM0", robot_id="so101") as arm:
    arm.enable()
    arm.set_servo_angle(
        angle=[0, -5, 5, 0, 0],
        is_radian=False,
        speed=10,
        mvacc=20,
        wait=True,
    )
    arm.relax()
