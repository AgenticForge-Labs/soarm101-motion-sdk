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
taught pose. It then preflights the elevated Cartesian endpoints and asks once for
confirmation before motion. Replay first makes one calibrated-Z clearance lift, then the
paper path is `A_UP -> B_UP -> C_UP -> D_UP -> CENTER_UP`.

The operator teaches A/B/C/D and one physical D_UP reference. Replay first reconstructs
the previously known-reachable endpoint branch, inverse-maps each endpoint into calibrated
workspace coordinates, preserves workspace X/Y, and sets workspace Z to the single
measured reference height. The corrected model-space targets must all pass read-only IK
preflight. No additional elevated teaching is required. The segment itself is still
executed with `move_linear()` and position-only IK.

`--replay` reuses the saved A/B/C/D plus D_UP teaching without touching the arm manually again and can begin
from an ordinary resting pose. Before entering the paper path, replay requests one 20 mm
calibrated-workspace-Z clearance rise by default. That path is preflighted at <=1 mm
spacing; X/Y must remain fixed in the calibrated model, physical Z must not descend,
sequential IK must remain continuous, and all solved joints must remain inside effective
limits. Measured workspace Z must land within 5 mm of the requested clearance target
before motion continues to A_UP. Because the generic model-frame floor is known to
disagree with the measured table, only that preflighted startup lift executes with the
coarse workspace check disabled. The paper
traversal endpoints continue to use destination-only coarse workspace validation. The
dynamic/joint/hardware safety stack remains active.
Use `--measure-only` to retain the non-moving behavior.

See [Workspace calibration](workspace-calibration.md) for the persisted contract,
quality gates, and provenance.

## Linear-motion smoothness

Live teleoperation is an important control comparison because it uses the same motors and
position loop without Cartesian IK. Teleoperation sends host-shaped joint samples with
Feetech speed_raw=0 (unrestricted) and acceleration_raw=254. Cartesian
`move_linear()` retains the responsive acceleration setting but now gives calibrated
Feetech joints proportional per-sample speed limits so synchronized writes target a common
arrival horizon. The SDK-wide planned-motion default remains 50 Hz, while the supervised
paper validation uses a 20 Hz host command cadence to match the known-smooth teleoperation
timing on this hardware.

The paper workflow retains a 1 mm Cartesian planning-density bound. Every emitted command
sample is a direct sequential-IK solution of a cosine-ramped trajectory with constant-speed
cruise when distance permits. Position-only paths ignore target orientation for timing. If a
single-start intermediate solve misses the unchanged 0.5 mm tolerance, that sample is
retried with multi-start before the path fails. If teleoperation remains smooth but linear
motion remains shaky at the same 20 Hz cadence, inspect encoder-quantized command deltas,
measured following error, and the planned joint derivatives before changing motor PID or
power settings.

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
