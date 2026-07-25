# Architecture

Dependency direction is strict:

```text
applications -> SOARM101 -> motion/kinematics/tools -> backend -> Feetech transport
```

Rules:

- Camera capture, tracking, OBS, and show orchestration remain outside this repository.
- The arm has five pose joints. The stock motor-6 gripper is an `SO101Gripper` tool.
- Tools define motion-relevant TCP transforms; camera operation belongs in Robo Cam.
- No LeRobot import exists in the runtime package.
- Hardware and simulation implement the same backend contract.
- Cartesian paths are validated before execution.
