# xArm API comparison

The SDK studies the UFACTORY xArm Python SDK as a developer-experience reference. It does not use xArm as a runtime dependency.

| xArm concept | SO-ARM101 API | Status |
|---|---|---|
| `connect()` | `connect()` | Implemented |
| `disconnect()` | `disconnect()` | Implemented |
| `get_servo_angle()` | `get_joint_positions()` | Core form implemented |
| `set_servo_angle()` | `move_joints()` | Direct form implemented |
| `move_gohome()` | `move_home()` | Planned |
| `set_position()` | `move_pose()` | Planned |
| linear Cartesian move | `move_linear()` | Planned |
| `emergency_stop()` | `stop()` | Initial software stop |
| controller GPIO | unsupported | Non-goal |
| Modbus | unsupported | Non-goal |

Similar naming does not imply equivalent industrial controller behavior or safety guarantees.
