# Development Plan

## Phase 0 — Repository foundation

- [x] Create Python package structure
- [x] Add Apache-2.0 license and attribution files
- [x] Add README and architecture documentation
- [x] Add backend protocol
- [x] Add mock backend
- [x] Add configuration and SDK-owned types
- [x] Add exception hierarchy
- [x] Add joint-limit validation
- [x] Add CI configuration
- [x] Add initial tests

Exit criteria still requiring execution in a development checkout:

- [ ] Confirm package builds in a clean Python 3.12 environment
- [ ] Confirm Ruff, mypy, and pytest pass

## Phase 1 — LeRobot hardware integration

- [ ] Construct the tested SO-101 follower through LeRobot
- [ ] Preserve existing LeRobot calibration
- [ ] Read calibrated joint positions
- [ ] Send direct joint targets
- [ ] Enable and disable torque
- [ ] Add communication diagnostics
- [ ] Test safe shutdown on physical hardware

## Phase 2 — Smooth joint motion

- [ ] Add timed joint moves
- [ ] Add minimum-jerk interpolation
- [ ] Add velocity and acceleration limits
- [ ] Add cancelable motion handles
- [ ] Add blocking and nonblocking commands
- [ ] Add command watchdog

## Phase 3 — Trajectories

- [ ] Record, save, load, edit, resample, and replay trajectories

## Phase 4 — Kinematics

- [ ] Define and verify geometry and coordinate frames
- [ ] Implement forward kinematics, Jacobian, and inverse kinematics

## Phase 5 — Cartesian motion

- [ ] Add pose motion, linear paths, workspace limits, and reachability checks

## Phase 6 — xArm-inspired compatibility

- [ ] Add documented aliases such as `set_servo_angle`, `set_position`, and `move_gohome`
- [ ] Add an optional result-code adapter and migration examples

## Phase 7 — Robo Cam integration

- [ ] Publish stable motion interfaces and a simulated integration path
