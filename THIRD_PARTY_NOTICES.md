# Third-party notices

## TheRobotStudio SO-ARM100 / SO-ARM101

License: Apache License 2.0.

The native kinematic model, joint limits, TCP transform, and packaged simplified simulation URDF are derived from the official `Simulation/SO101/so101_new_calib.urdf`. The simplified URDF omits the original meshes and uses primitive visual geometry.
The GUI presentation additionally references the official mesh visual origins from the same
SO-101 model (revision `385e8d7c68e24945df6c60d9bd68837a4b7411ae`) so printed members
keep their physical offsets and motor cases keep their physical orientation. The nominal jaw
outline also uses the official moving-jaw mesh extents and visual origin offset from
`Simulation/SO101/assets/moving_jaw_so101_v1.stl`. Original mesh bytes are not vendored;
these shapes are schematic presentation, not collision meshes.

## Hugging Face LeRobot

License: Apache License 2.0.

Used as a behavioral and calibration-format reference. LeRobot is not a runtime dependency. Calibration conversion follows its `MotorCalibration` fields and normalization conventions.

## Feetech STS3215 / FTServo Python SDK

The GUI uses the manufacturer-published STS3215 outside dimensions (45.23 x 24.73 x 35 mm)
for presentation-only motor-case envelopes. Runtime hardware communication is provided by the
separately installed `ftservo-python-sdk` package (MIT). No SDK source or servo CAD is vendored here.

## UFACTORY xArm Python SDK

License: BSD 3-Clause.

Used as an API and developer-experience reference. It is not a runtime dependency, and this project is not affiliated with UFACTORY.
