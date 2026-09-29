# Physical kinematics validation

The native FK/IK model comes from the official SO-101 calibrated URDF, but printed assembly tolerances, horn indexing, backlash, and TCP mounting can create measurable error. Validate the exact arm before relying on larger Cartesian moves.

## Recommended procedure

1. Secure the base and define a repeatable base coordinate origin.
2. Leave torque disabled and position the arm manually at a stable checkpoint.
3. Measure the active TCP in millimeters using a ruler, square, jig, or camera calibration target.
4. Record the comparison:

```bash
soarm101 kinematics-check \
  --port PORT --robot-id ID \
  --sample home \
  --x-mm X --y-mm Y --z-mm Z \
  --output kinematics-validation.jsonl
```

5. Capture at least five poses spanning the useful workspace, not only the home pose.
6. Review the error vectors and norm. Large systematic offsets usually indicate TCP definition or base-frame error; pose-dependent errors suggest link dimensions, horn indexing, calibration, or compliance.
7. Keep early Cartesian moves to 2–5 mm until results are understood.

The command is read-only and never enables torque. It records predicted position, measured position, joint angles, component error, and total position error as JSON Lines for later analysis.


## Three-corner paper-frame validation

After the mechanical calibration, one-joint direction checks, and initial gripper checks
have passed, `examples/paper_corner_cartesian_test.py` provides a simple physical
Cartesian-space validation using a known rectangular sheet. US Letter defaults are
215.9 x 279.4 mm.

The script:

1. briefly enables torque to open the gripper fully, then relaxes the arm;
2. asks the operator to place the same lower gripper finger on corner A, the adjacent
   width corner B, and the adjacent height corner C, pressing Enter at each corner;
3. records the FK TCP XYZ/RPY and joint state at each touch;
4. reports measured width, height, corner angle, paper-plane axes, and probe-orientation
   drift;
5. derives an orthonormal paper frame and predicts the unseen fourth corner from the
   known paper dimensions;
6. with torque still off, preflights exact-orientation IK for the complete test;
7. after an explicit confirmation, lifts off the paper, traverses above the predicted
   fourth corner, points just above it, and then visits several +Z heights at that same
   paper-frame X/Y location.

Run it from the repository root:

```bash
python examples/paper_corner_cartesian_test.py
```

The saved workstation follower is used when `--port` is omitted. The default fourth-
corner point stops 2 mm above the paper and the default Z test visits 25, 50, and
100 mm above the far corner.

The stock SDK TCP is the modeled gripper TCP, not the physical lower-finger tip. Using
the lower finger as a probe is therefore valid only when its pose relative to the modeled
TCP remains effectively constant. The script reports A/B/C orientation drift and refuses
autonomous motion above the configured drift threshold unless the override is deliberate.
A future calibrated lower-finger TCP would remove this approximation.
