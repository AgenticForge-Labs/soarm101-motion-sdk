# Changelog

## Unreleased

- Replaced the LeRobot runtime plan with direct official Feetech SDK control.
- Added calibration compatibility, stock-gripper tooling, FK, bounded IK, smooth joint and linear motion, diagnostics, CLI workflows, and simulation.
- Hardened torque enable by latching measured positions before energizing the servos.
- Unified blocking and nonblocking motion cancellation and made software stop wait for the active motion task to terminate.
- Added complete command-rate trajectory prevalidation, joint speed/acceleration checks, and feedback-based completion timeouts.
- Fixed anti-parallel camera and approach-axis IK constraints.
- Made calibration transactional with EEPROM rollback on failure or interruption.
- Serialized Feetech transport access and made torque transitions transactional.
- Added in-motion fault, following-error, direction, and command-deadline monitoring.
- Added absolute motion safety ceilings and independent compatible-orientation residuals.
- Made stock-gripper operations cancellable through the arm-level stop lifecycle.
