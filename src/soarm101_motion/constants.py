"""SO-ARM101 physical constants and canonical names.

Joint origins and limits are derived from TheRobotStudio's Apache-2.0
``Simulation/SO101/so101_new_calib.urdf``. The stock follower uses six
STS3215 servos: five pose joints plus one gripper actuator.
"""

from __future__ import annotations

from collections import OrderedDict
from math import pi

ARM_JOINTS: tuple[str, ...] = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
)
STOCK_GRIPPER = "so101_gripper"
ALL_MOTORS: tuple[str, ...] = (*ARM_JOINTS, STOCK_GRIPPER)

MOTOR_IDS: OrderedDict[str, int] = OrderedDict(
    (
        ("shoulder_pan", 1),
        ("shoulder_lift", 2),
        ("elbow_flex", 3),
        ("wrist_flex", 4),
        ("wrist_roll", 5),
        (STOCK_GRIPPER, 6),
    )
)

EXPECTED_MODEL_NUMBER = 777
ENCODER_RESOLUTION = 4096
ENCODER_MAX = ENCODER_RESOLUTION - 1
HALF_TURN = ENCODER_MAX // 2

# Verified against the official SO-101 calibrated URDF. These are mechanical
# planning limits, not a certification that every printed arm can safely reach
# every boundary under load.
JOINT_LIMITS: dict[str, tuple[float, float]] = {
    "shoulder_pan": (-1.91986, 1.91986),
    "shoulder_lift": (-1.74533, 1.74533),
    "elbow_flex": (-1.69, 1.69),
    "wrist_flex": (-1.65806, 1.65806),
    "wrist_roll": (-2.74385, 2.84121),
}

HOME_JOINTS: dict[str, float] = {joint: 0.0 for joint in ARM_JOINTS}

# Conservative host-side defaults. These are intentionally below the values
# commonly used by direct teleoperation loops.
DEFAULT_JOINT_SPEED_RAD_S = 0.45
DEFAULT_JOINT_ACCEL_RAD_S2 = 1.2
DEFAULT_LINEAR_SPEED_M_S = 0.03
DEFAULT_LINEAR_ACCEL_M_S2 = 0.10
DEFAULT_COMMAND_FREQUENCY_HZ = 50.0
# Conservative initial rate for physical leader→follower teleoperation.  The normal
# trajectory clock remains 50 Hz; streaming starts slower because each sample currently
# performs synchronous hardware safety/feedback reads on the serial bus.
DEFAULT_TELEOP_STREAM_FREQUENCY_HZ = 10.0
DEFAULT_MAX_COMMAND_STEP_RAD = 5.0 * pi / 180.0

# Feetech STS3215 control table. Address and byte width.
STS3215_REGISTERS: dict[str, tuple[int, int]] = {
    "Firmware_Major_Version": (0, 1),
    "Firmware_Minor_Version": (1, 1),
    "Model_Number": (3, 2),
    "ID": (5, 1),
    "Baud_Rate": (6, 1),
    "Return_Delay_Time": (7, 1),
    "Response_Status_Level": (8, 1),
    "Min_Position_Limit": (9, 2),
    "Max_Position_Limit": (11, 2),
    "Max_Temperature_Limit": (13, 1),
    "Max_Voltage_Limit": (14, 1),
    "Min_Voltage_Limit": (15, 1),
    "Max_Torque_Limit": (16, 2),
    "Phase": (18, 1),
    "P_Coefficient": (21, 1),
    "D_Coefficient": (22, 1),
    "I_Coefficient": (23, 1),
    "Protection_Current": (28, 2),
    "Homing_Offset": (31, 2),
    "Operating_Mode": (33, 1),
    "Overload_Torque": (36, 1),
    "Torque_Enable": (40, 1),
    "Acceleration": (41, 1),
    "Goal_Position": (42, 2),
    "Goal_Time": (44, 2),
    "Goal_Velocity": (46, 2),
    "Torque_Limit": (48, 2),
    "Lock": (55, 1),
    "Present_Position": (56, 2),
    "Present_Velocity": (58, 2),
    "Present_Load": (60, 2),
    "Present_Voltage": (62, 1),
    "Present_Temperature": (63, 1),
    "Status": (65, 1),
    "Moving": (66, 1),
    "Present_Current": (69, 2),
    "Maximum_Velocity_Limit": (84, 1),
    "Maximum_Acceleration": (85, 1),
}
