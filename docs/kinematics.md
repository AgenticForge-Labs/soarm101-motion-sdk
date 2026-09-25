# Kinematics and IK

The native model reproduces the five revolute-joint origins, axes, calibrated limits, and stock gripper TCP from the official SO-101 URDF. It does not require ROS or a URDF parser at runtime.

Inverse kinematics uses bounded `scipy.optimize.least_squares`. The current or previous joint configuration is the seed, which preserves continuity during linear motion. Planned Cartesian paths use a configurable 0.5 mm position tolerance by default; this is a solver acceptance threshold, not a claim of 0.5 mm physical accuracy.

Orientation modes:

- `exact`: require full requested orientation within tolerance when the pose lies on the five-axis reachable manifold.
- `compatible`: prioritize position and TCP approach-axis alignment while weakly preferring the requested lateral axis.
- `position_only`: constrain XYZ and allow orientation to float.
- `look_at`: place the TCP and aim its +Z axis at a target point, leaving roll unconstrained.

`move_linear()` interpolates Cartesian position and orientation, solves IK sequentially, rejects discontinuities, computes a safe total duration, then uniformly resamples the joint path at the command rate.

The Manual GUI's joint-center view is generated from this same native model rather than a second visual-only arm definition. It starts in an orthographic X/Z side view and shows joint axes and the TCP, not the printed link housings or gripper mesh. Dragging rotates the view; double-clicking restores the side view. During a jog, the GUI marks the requested TCP target and logs the requested-versus-achieved Cartesian pose. The physical validation sequence in `TESTING.md` uses those diagnostics to distinguish a GUI/planner axis error from a calibration or physical joint-direction mismatch.
