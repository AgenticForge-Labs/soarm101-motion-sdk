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
