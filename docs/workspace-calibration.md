# Workspace calibration

Motor calibration and workspace calibration answer different questions.

- **Motor calibration** maps servo encoder readings into joint coordinates.
- **Workspace calibration** records how a particular physical work surface and a
  manually demonstrated physical-UP direction map into the SDK's kinematic model on
  one arm/setup.

Workspace calibration is machine-local evidence. It does not change motor EEPROM or
rewrite the arm's mechanical calibration.

## Why this exists

Hardware testing showed that the generic model floor and model +Z direction cannot be
assumed to represent the physical table and physical up on every assembled setup.

In one paper test, four manually touched table corners were nearly coplanar in model
coordinates, yet a commanded model +Z hover physically moved laterally and contacted
the table before settling timed out. That result invalidated the earlier assumption that
"model +Z" could be treated as "physical up" merely because the paper looked horizontal
in FK.

The replacement workflow therefore measures physical up explicitly instead of inferring it.

## Paper workspace workflow

Run:

```bash
python examples/paper_workspace_calibration.py --reference-height-mm 50
```

The historical `--hover-height-mm` spelling is accepted as an alias, but it no longer
commands motion. The old `examples/paper_four_corner_hover_demo.py` filename is also
retained only as a compatibility wrapper around this calibration workflow.

The operator teaches, with the arm relaxed:

```text
D ---------------- C
|                  |
|                  |
A ---------------- B
```

1. A: lower-left paper corner.
2. B: adjacent short/width corner.
3. C: next long/height corner.
4. D: remaining short/width corner.
5. UP: the same fixed lower finger at a physically measured height above D.

For UP, use a ruler, rigid spacer, paper edge, gauge block, or another physical reference.
Allow the wrist/tool orientation to change naturally as needed to put the fixed lower
finger at the measured physical point. The workflow performs no autonomous Cartesian
arm motion.

## What is fitted

The physical paper coordinates are defined as:

```text
A  = (0,      0,       0)
B  = (width,  0,       0)
C  = (width,  height,  0)
D  = (0,      height,  0)
UP = (0,      height,  reference_height)
```

A local affine transform is fitted from those physical coordinates into model/base
coordinates:

```text
model_position = origin + M @ physical_paper_position
```

The calibration records:

- the fitted table plane point and normal in model coordinates;
- local model scale along physical paper X, Y, and UP;
- the model angle between physical X and Y;
- the angle between the manually demonstrated physical-UP direction and the fitted
  table-plane normal;
- table-plane and affine-fit residuals;
- the four model-space paper corners and the model-space UP reference; and
- the active motor-calibration ID.

The calibration is content-addressed with a workspace ID and stored by default at:

```text
~/.config/soarm101/workspace/<robot-id>.json
```

The diagnostic report is also written in the working directory.

## Measurement acceptance gate

The workflow writes the measured workspace calibration when:

- the four taught corners form a sufficiently consistent plane;
- the full physical-to-model affine fit has a sufficiently small residual;
- the 3-D linear mapping is well-conditioned enough to invert reliably; and
- the model displacement produced by the measured UP height has a plausible scale rather
  than collapsing toward zero or expanding implausibly.

D -> UP tool-orientation drift is recorded as diagnostic evidence only. With this 5-DOF
arm and a fixed fingertip used as the physical probe, changing wrist/tool orientation can
be required simply to place that fingertip at the desired physical point. It therefore
does not participate in workspace-measurement acceptance.

The angle between the mapped physical-UP vector and the Euclidean normal of the
model-space paper plane is retained as a diagnostic, **not** as an acceptance condition.
A skewed local affine mapping can legitimately make those directions non-parallel; that
skew is part of what this calibration is intended to measure.

A newly measured calibration is initially saved with:

```text
motion_validation_status = "unvalidated"
```

The same workflow may then run the supervised demonstration
`D_UP -> A_UP -> B_UP -> C_UP -> D_UP -> CENTER_UP`, all at the measured physical
reference height. For the powered demonstration, the global affine fit is **not extrapolated upward**.
Instead, the directly measured model-space D→UP displacement is added to each directly
measured A/B/C/D corner. CENTER_UP is the mean of those four elevated measured corners.
This keeps the powered target construction anchored to actual taught observations rather
than asking the affine approximation to predict unmeasured elevated corners. Position-only
IK is used; these are not model +Z offsets. The report records
preflight results and achieved TCP evidence. This demonstration is still narrower than
general autonomous Cartesian validation.

## Relationship to the coarse floor guard

The existing coarse workspace guard still uses the SDK's current generic model envelope.
The new workspace artifact is the source of truth for future table-aware validation, but
it is **not yet used to authorize Cartesian motion automatically**.

The next validation stage should use the measured mapping to design very small,
supervised physical-direction tests. Only after those pass should the runtime floor guard
or higher-level Cartesian capabilities consume a workspace calibration as an execution
authority.

Do not lower/disable the generic floor or use a paper-derived model normal as a substitute
for this physical-UP measurement.
