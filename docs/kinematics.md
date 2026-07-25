# Kinematics and IK

The native model reproduces the five revolute-joint origins, axes, calibrated limits, and stock gripper TCP from the official SO-101 URDF. It does not require ROS or a URDF parser at runtime.

Inverse kinematics uses bounded `scipy.optimize.least_squares`. The current or previous joint configuration is the seed, which preserves continuity during linear motion.

Orientation modes:

- `exact`: require full requested orientation within tolerance when the pose lies on the five-axis reachable manifold.
- `compatible`: prioritize position and TCP approach-axis alignment while weakly preferring the requested lateral axis.
- `position_only`: constrain XYZ and allow orientation to float.
- `look_at`: place the TCP and aim its +Z axis at a target point, leaving roll unconstrained.

`move_linear()` interpolates Cartesian position and orientation, solves IK sequentially, rejects discontinuities, computes a safe total duration, then uniformly resamples the joint path at the command rate.
