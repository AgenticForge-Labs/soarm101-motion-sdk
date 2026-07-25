# Development plan

## Implemented foundation

- [x] Direct official Feetech SDK hardware backend
- [x] STS3215 IDs, register table, model verification, synchronous commands
- [x] Native and LeRobot-compatible calibration storage/discovery
- [x] Five-joint arm and `SO101Gripper` tool separation
- [x] Official-URDF FK model and joint limits
- [x] Bounded numerical IK with five-axis task modes
- [x] Smooth minimum-jerk joint movement
- [x] Preplanned Cartesian linear movement
- [x] Blocking and nonblocking motion handles with unified cancellation
- [x] Torque-safe enable with present-position goal latching
- [x] Command-rate trajectory speed, acceleration, step, and limit validation
- [x] Feedback-based motion completion and timeout reporting
- [x] Transactional calibration with EEPROM rollback
- [x] Deterministic simulator and optional PyBullet visualization
- [x] Diagnostics, guarded CLI movement, and interactive calibration
- [x] Unit, integration, kinematics, simulation, and fake-Feetech tests

## Physical validation gate

- [ ] Run port discovery and six-motor model check on the user's arm
- [ ] Import or perform calibration and verify all directions
- [ ] Run ±5° joint smoke test with no payload
- [ ] Validate stop, completion timeout, and torque-safe enable on hardware
- [ ] Validate FK against measured TCP positions
- [ ] Validate low-speed linear paths and tune tolerances
- [ ] Tag first hardware-validated alpha release

## Later

- [ ] Trajectory recording/editing/playback formats
- [ ] Parallel gripper implementation
- [ ] Tool assemblies with camera and gripper TCPs
- [ ] Optional collision geometry and self-collision checks
- [ ] Robo Cam integration examples
