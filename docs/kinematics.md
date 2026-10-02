# Kinematics and IK

The native model reproduces the five revolute-joint origins, axes, nominal planning limits, and stock gripper TCP from the official SO-101 URDF. It does not require ROS or a URDF parser at runtime.

The URDF joint limits are the generic model fallback/reference. On a calibrated real arm,
runtime joint authority follows the measured mechanical-stop ranges with a 1° inset from
each pose-joint stop by default. This lets the kinematic solver use the arm's measured
travel rather than clipping every assembly to the generic URDF range, while still keeping
normal commands off the measured stop itself.

Inverse kinematics uses bounded `scipy.optimize.least_squares`. The current or previous joint configuration is the seed, which preserves continuity during linear motion. Planned Cartesian paths use a configurable 0.5 mm position tolerance by default; this is a solver acceptance threshold, not a claim of 0.5 mm physical accuracy. Continuity and joint-centering are soft regularizers only: if they leave an otherwise reachable target outside the hard task tolerance, the solver performs a bounded task-space-only refinement from the regularized candidates rather than relaxing the tolerance.

Orientation modes:

- `exact`: require full requested orientation within tolerance when the pose lies on the five-axis reachable manifold.
- `compatible`: prioritize position and TCP approach-axis alignment while weakly preferring the requested lateral axis.
- `position_only`: constrain XYZ and allow orientation to float.
- `look_at`: place the TCP and aim its +Z axis at a target point, leaving roll unconstrained.

`move_linear()` interpolates Cartesian position and orientation, solves IK sequentially at the command rate, and rejects discontinuities. For `position_only` paths, a deterministic joint-space filter may be used only to create smoother IK seeds; every interior Cartesian sample is then re-solved at the same hard tolerance, and the refined sequence is kept only when discrete joint jerk is lower. If forward sequential IK still hits a numerical local minimum, the planner may solve the same samples backward from a reachable endpoint solution. A caller-provided endpoint seed is only a boundary-condition hint: the reverse path must satisfy the unchanged hard tolerance at every sample and reconnect continuously to the measured start. Cartesian geometry remains the source of truth.

The Manual GUI's joint-center view is generated from this same native model rather than a second visual-only arm definition. It starts in an orthographic X/Z side view and shows joint axes and the TCP, not the printed link housings or gripper mesh. Dragging rotates the view; double-clicking restores the side view. During a jog, the GUI marks the requested TCP target and logs the requested-versus-achieved Cartesian pose. The physical validation sequence in `TESTING.md` uses those diagnostics to distinguish a GUI/planner axis error from a calibration or physical joint-direction mismatch.
