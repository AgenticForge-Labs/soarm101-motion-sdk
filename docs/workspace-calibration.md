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

For UP, use a ruler, rigid spacer, gauge block, or another physical reference. Keep the
tool orientation as close to the D orientation as practical. The workflow performs no
autonomous Cartesian arm motion.

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

## Activation gate

The workflow only writes the authoritative workspace calibration when:

- the four taught corners form a sufficiently consistent plane;
- the manually demonstrated physical-UP direction agrees with that plane's normal within
  the configured tolerance; and
- D -> UP tool-orientation drift is small enough that the fixed-finger probe approximation
  is not obviously corrupted by a changing TCP offset.

Even when saved, a newly measured calibration has:

```text
motion_validation_status = "unvalidated"
```

That is deliberate. Measurement of the workspace frame is not the same thing as proving
that autonomous Cartesian interpolation is physically safe.

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
