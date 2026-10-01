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

The workflow performs no autonomous Cartesian arm motion.

See [Workspace calibration](workspace-calibration.md) for the persisted contract,
quality gates, and provenance.

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
