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

The required three-corner layout is:

```text
C ---------------- D  (predicted; do not touch)
|                  |
|                  |
A ---------------- B
```

A->B is the 215.9 mm short/width edge. A->C is the 279.4 mm long/height edge and
**C must be on the same side of the sheet as A**. If the operator goes A->B and then
up the right side, that point is D, the far corner, and the rectangle-angle check is
invalid.

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

## Four-corner hover diagnostic

`examples/paper_four_corner_hover_demo.py` is a supervised motion diagnostic for cases
where the operator wants to observe slow Cartesian motion without predicting an unseen
corner. It teaches every physical corner manually in clockwise order:

```text
D ---------------- C
|                  |
|                  |
A ---------------- B
```

The operator may therefore move A->B left-to-right and then B->C up the right side.
After all four torque-off captures, the script reports the model-space paper distances
for diagnosis but does not use them to alter any target. Each hover target is simply
50 mm by default along SDK base +Z from that corner's captured model TCP, retaining that
corner's captured orientation as the compatible-IK reference.

All four hover targets are solved with torque off first. Powered execution has two
separate confirmations: first only the lift from the current D touch to the D hover;
after the operator verifies that motion went upward and is clear of the table, the
script traverses D->A->B->C->D at the hover height.

The manually taught paper touch may place the modeled TCP below the SDK's generic
base-Z=0 coarse floor even though the physical finger is safely resting on the paper.
For the **first lift only**, the script opts into guarded floor recovery: any point that
starts below the configured floor must progress upward without a meaningful downward
dip, points that start above the floor must remain above it, the normal reach/base/
self-clearance checks stay active, and the lift must finish entirely inside the ordinary
workspace envelope. Subsequent perimeter moves use the normal floor rule with no recovery
exception.

Each `move_linear()` segment still uses the normal guarded planner and is fully planned
before that segment sends motor commands. The demo never disables workspace checks or
bypasses joint, calibration, following-error, fault, effort, or motion-planning guards.

This demo is deliberately **not** evidence that Cartesian kinematics are calibrated,
and it does not replace the three-corner validation gate. Keep physical power immediately
reachable and stop if the first lift is not physically upward and clear.

