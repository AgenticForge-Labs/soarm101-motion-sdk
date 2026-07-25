# Architecture

Dependency direction is strictly downward:

```text
Applications
    -> SOARM101 public API
    -> validation and motion services
    -> RobotBackend protocol
    -> Mock or LeRobot backend
    -> LeRobot / Feetech hardware
```

Rules:

- Applications never import LeRobot through this SDK.
- Core modules never import a concrete backend.
- Only the LeRobot adapter imports LeRobot.
- Camera and video behavior remain outside this repository.
- Kinematics and trajectory systems must remain backend-independent.
- xArm-inspired compatibility is layered above the core API rather than embedded in it.
