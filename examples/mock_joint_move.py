from soarm101_motion import SOARM101
from soarm101_motion.backends import MockSOARM101Backend


def main() -> None:
    backend = MockSOARM101Backend()
    arm = SOARM101(backend=backend)
    arm.connect()
    print("Initial positions:", arm.get_joint_positions())
    arm.move_joints({"shoulder_pan": 0.25, "shoulder_lift": -0.35})
    print("Updated positions:", arm.get_joint_positions())
    arm.relax()
    arm.disconnect()


if __name__ == "__main__":
    main()
