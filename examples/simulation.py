from soarm101_motion import SOARM101

with SOARM101.simulated() as arm:
    arm.enable()
    arm.move_joints([0.2, -0.4, 0.6, 0.1, 0.0])
    arm.tool.open()
    print(arm.get_position().xyz_rpy())
