# Physical kinematics validation

The native FK/IK model comes from the official SO-101 calibrated URDF, but printed
assembly tolerances, horn indexing, backlash, TCP mounting, calibration conventions,
and frame interpretation can create measurable error. Validate the exact arm before
relying on Cartesian motion.

## Measured FK sanity checks

1. Secure the base and define a repeatable physical reference.
2. Leave torque disabled and position the arm manually at a stable checkpoint.
3. Measure the active TCP in millimeters using a ruler, square, jig, or camera
   calibration target.
4. Record the comparison:

```bash
soarm101 kinematics-check \
  --port PORT --robot-id ID \
  --sample home \
  --x-mm X --y-mm Y --z-mm Z \
  --output kinematics-validation.jsonl
```

5. Capture several poses spanning the useful workspace.
6. Review the error vectors and norm. Large systematic offsets usually indicate TCP or
   base-frame error; pose-dependent errors suggest link dimensions, horn indexing,
   calibration, compliance, or another kinematic mismatch.

The command is read-only and never enables torque.

## Three-corner paper geometry check

`examples/paper_corner_cartesian_test.py` is now a read-only geometry diagnostic after
the brief powered gripper-open step. It does **not** command Cartesian arm motion.

Required layout:

```text
C ---------------- D  (predicted; do not touch)
|                  |
|                  |
A ---------------- B
```

A->B is the 215.9 mm short/width edge of US Letter. A->C is the 279.4 mm
long/height edge and C must be on the same side of the sheet as A.

The script records the model TCP at A/B/C, reports the measured edge lengths and corner
angle, and predicts D in model coordinates. Its model-space paper normal is diagnostic
only. It is not accepted as demonstrated physical up.

This change follows a physical test in which a paper/model normal looked plausible
numerically but a commanded "up" hover moved laterally and contacted the table.

## Four-corner workspace calibration

Use:

```bash
python examples/paper_workspace_calibration.py --reference-height-mm 50
```

This is **manual workspace calibration**, not a hover demo. The historical
`examples/paper_four_corner_hover_demo.py` filename remains only as a safe compatibility
wrapper. The old `--hover-height-mm` spelling also remains as an alias for
`--reference-height-mm`; neither compatibility path commands Cartesian motion.

Teach:

```text
D ---------------- C
|                  |
|                  |
A ---------------- B
```

Then manually place the same fixed lower finger at a physically measured height above D.
Use a ruler, rigid spacer, gauge block, or another physical reference and keep the tool
orientation as close to D as practical.

Those five physical/model correspondences fit a local affine workspace mapping and a
table plane. The calibration is tied to the current motor-calibration ID and is stored,
when quality gates pass, under:

```text
~/.config/soarm101/workspace/<robot-id>.json
```

After the final UP sample, the workflow counts down and enables torque to hold that
taught pose. It preflights the leveled A_UP/B_UP/C_UP/D_UP/CENTER_UP endpoints and asks
once for confirmation before motion. Replay first makes one separately preflighted
calibrated-workspace-Z clearance move, then executes
`A_UP -> B_UP -> C_UP -> D_UP -> CENTER_UP`.

The single D_UP measurement defines the replay height. For each paper endpoint, software
keeps its calibrated workspace X/Y and replaces workspace Z with that same measured
reference height before mapping the point back into model coordinates. CENTER_UP is the
true calibrated paper midpoint. Because the workspace mapping is affine, straight
model-space interpolation between two equal-workspace-Z leveled endpoints maps back to a
straight constant-height line in the calibrated workspace.

The elevated segments are executed with the SDK's Cartesian `move_linear()` primitive and
position-only IK at the paper workflow's 20 Hz host cadence. Endpoint IK is read-only
preflighted before powered motion; each `move_linear()` segment still plans and validates
its full sequential-IK trajectory before issuing motor commands. The generic model
workspace is used only as a destination sanity check for the paper segments because its
table floor is known to disagree with the measured workspace. Joint limits, command
step/rate/acceleration, following error, motor faults, effort/contact guards, communication
checks, and motion timeout remain active.

`--replay` reuses the saved A/B/C/D plus D_UP teaching without touching the paper again
and can begin from an ordinary resting pose. Use `--measure-only` to retain the non-moving
behavior.

See [Workspace calibration](workspace-calibration.md) for the persisted contract,
quality gates, and provenance.

## Linear-motion smoothness

Live teleoperation is the control baseline because it uses the same motors and position
loop without Cartesian IK. Teleoperation streams host-limited joint targets at 20 Hz with
Feetech `speed_raw=0` (maximum tracking authority) and
`acceleration_raw=254`.

Planned Cartesian `move_linear()` now uses the same servo-side tracking profile. The host
trajectory remains authoritative for Cartesian and joint speed/acceleration; the servo no
longer receives an additional tiny per-sample speed cap. Hardware testing showed that the
old synchronized-arrival speed throttling could make gravity-loaded Cartesian motion visibly
stick-slip/shake even though teleoperation on the same arm was smooth.

The paper workflow also uses a 20 Hz host cadence and a 1 mm Cartesian planning-density
bound. Every emitted command sample is a sequential-IK solution of the Cartesian
trajectory. The #73 launch-only experiment did not remove the visible shake. After #74,
hardware was still very shaky and A_UP->B_UP again failed before motion, now at 0.794 mm
against the unchanged 0.5 mm tolerance.

Review of the public managed controller confirmed that real Cartesian execution was
already using the documented teleoperation baseline (`speed_raw=0`,
`acceleration_raw=254`). The visible shake therefore remains unresolved and is not
evidence for another servo-profile change.

Position-only paths retain deterministic smooth-seed reprojection. In addition, if forward
sequential IK hits a numerical pocket, the planner can use an exact reachable endpoint
solution as a second boundary condition and solve the same Cartesian samples backward.
The paper workflow reuses its read-only endpoint-preflight joint solution for this purpose.
A reverse path is accepted only if it reconnects continuously to the measured start and
every sample satisfies the same hard tolerance, joint limits, and dynamic guards.
Separately, when soft IK continuity/joint-centering regularization prevents a geometrically
reachable sample from meeting the hard tolerance, a task-space-only refinement is attempted
without relaxing that tolerance.

Hardware #75 testing also showed that the endpoint-seeded reverse fallback does not
resolve A_UP->B_UP: forward and reverse planning converged to essentially the same
0.648/0.646 mm miss. That pattern can indicate a true local reachability/singularity or
joint-limit boundary rather than a seed-direction problem. The paper workflow therefore
preflights every complete elevated segment read-only before powered traversal and reports
the exact failing sample, signed XYZ residual, positional Jacobian conditioning, and
nearest effective joint-limit margin.

Because `move_linear()` also remains very shaky despite the existing teleoperation-style
servo profile, once a complete paper path is actually feasible the next comparison should
be planned joint derivatives, encoder-quantized command deltas, measured following error,
and actual cycle timing; that would isolate remaining IK/Jacobian/quantization effects from
servo tracking.

## Current hardware finding

On the tested follower, a four-corner paper capture produced a model-space paper plane
that was nearly horizontal, yet a requested model +Z hover physically traveled roughly
along the table and contacted it. The motion then failed to settle and the SDK relaxed
the arm.

That means the present issue is **not just a table-height offset**. The physical/model
Cartesian direction mapping itself still requires validation.

Therefore:

- do not lower the generic model floor merely to make a paper touch pass;
- do not use base +Z or a paper-derived model normal as physical up;
- do not resume autonomous paper hover/traverse motion yet; and
- use the manually measured UP reference to characterize the local mapping first.

## Next powered Cartesian gate

Powered Cartesian testing remains blocked until the workspace measurement has been
reviewed and a separate supervised physical-direction validation is designed.

That later validation should start with very small motion, explicit before/after
measurement, immediate physical-power access, and no assumption that a model axis maps
directly to a physical world axis.
