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
- GUI and CLI features call the same SDK operations and saved libraries; the GUI owns persistent hardware sessions rather than launching CLI subprocesses.
- The GUI term **Program** is a presentation layer over the persisted `MotionSequence`
  model and `SequenceRunner`. Saved-position programs therefore do not introduce a
  second execution engine or bypass sequence provenance/safety checks.
- The GUI owns one persistent follower-status sidebar outside the task tabs. Its primary
  kinematic model is always the measured follower state when connected; active workflows
  may add a secondary leader/saved/recorded/program ghost without replacing that primary
  state.
- Kinematic GUI previews consume the same `SO101KinematicModel` used by planning; ghost
  overlays are visualization only and never authorize or execute motion.
- Physical motion artifacts carry calibration provenance and must fail closed on missing or mismatched target calibration during real-arm replay.
- Planned motion and live streaming share the core joint/rate/following-error/fault/effort safety stack, while live-stream workspace checks remain opt-in until the table frame and tool geometry are calibrated.
