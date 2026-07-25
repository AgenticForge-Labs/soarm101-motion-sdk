# Third-party notices

## TheRobotStudio SO-ARM100 / SO-ARM101

License: Apache License 2.0.

The native kinematic model, joint limits, TCP transform, and packaged simplified simulation URDF are derived from the official `Simulation/SO101/so101_new_calib.urdf`. The simplified URDF omits the original meshes and uses primitive visual geometry.

## Hugging Face LeRobot

License: Apache License 2.0.

Used as a behavioral and calibration-format reference. LeRobot is not a runtime dependency. Calibration conversion follows its `MotorCalibration` fields and normalization conventions.

## Feetech FTServo Python SDK

License: MIT.

Runtime hardware communication is provided by the separately installed `ftservo-python-sdk` package. No SDK source is vendored here.

## UFACTORY xArm Python SDK

License: BSD 3-Clause.

Used as an API and developer-experience reference. It is not a runtime dependency, and this project is not affiliated with UFACTORY.
