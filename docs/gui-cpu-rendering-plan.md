# CPU-only viewer optimization handoff

## Objective and roles

Make orbiting, zooming and measured-pose updates responsive on the operator's
consumer CPU without a GPU requirement. Terra 6.1 checks the approach, handles
difficult escalations and reviews the result. Luna performs most profiling,
implementation and software validation. Return to Luna after each escalation.

An implementation delegate is already running; do not start a competing editor.
Terra's planning pass is read-only. Carry any findings into the implementation
before final review. Preserve unrelated working-tree changes, especially the
concurrent choreography work. Do not commit, push or operate hardware.

## Baseline and measurements

The current mesh set has roughly 48,000 triangle instances per pose. Prior local
offscreen observations were about 0.4 seconds for a subsequent frame and six
seconds for first display; reproduce these rather than treating them as a formal
benchmark. Record CPU/runtime, viewport dimensions, pose sequence and camera
sequence. Measure cold startup separately from warm orbit, zoom, unchanged
repaint and changing measured poses. Report median and slow-frame latency.

Use the same workloads before and after, including the actual sidebar size and
a 480 by 480 standalone viewport. Aim for warm interaction near 30 frames/second
where feasible, and at least a substantial measured latency reduction. Report
the achieved numbers honestly if the target is not met. Avoid brittle timing
thresholds in CI.

## Implementation order

1. Profile range-envelope construction, mesh loading/transformation, projection,
   Python/Qt object creation and rasterization independently.
2. Vectorize binary STL loading. Cache immutable local triangles, normals and
   fixed visual transforms. Cache transformed geometry for unchanged joint and
   jaw state with explicit invalidation.
3. Remove redundant per-triangle allocations and color calculations. Precompute
   shading palettes, reuse grouped paths, and batch projection and classification.
   Cull hidden/back-facing faces only after verifying winding and visual results.
4. If needed, add a valid lower-detail mesh for interaction and live updates,
   produced by topology-preserving decimation. Never drop arbitrary facets.
   Restore detailed still rendering after interaction. Preserve all moving parts.
5. Reuse range-envelope results and keep expensive calculations out of repeated
   paint calls. Coalesce visual updates to the newest pose and cache unchanged
   frames where helpful. Any visual rate cap must leave worker motion, feedback,
   heartbeat and STOP processing independent.

Do not introduce mandatory OpenGL, GPU, external viewer processes or heavy runtime
dependencies. Do not change motion or safety behavior as a rendering optimization.

## Correctness and acceptance

- Meshes, joint markers, TCP and ghost targets use the same corrected projection.
- SDK link frames and stock jaw mapping remain authoritative.
- Inspect actual offscreen pixels for side, isometric, folded and extended poses;
  test orbit, resize, zoom and open/closed jaws. No missing surfaces, flattening,
  disconnected parts or stale poses after caching.
- Test cache invalidation on pose, jaw, camera, viewport, TCP and palette changes.
- Retain stable Fit Range and distinct Fit Pose behavior. Document the sampled
  viewing envelope without claiming collision clearance.
- Preserve asset provenance, license and pinned URDF; record derivative LOD
  processing and verify resources from a built wheel outside the checkout.
- Run relevant GUI geometry, mesh and session tests, lint and diff checks.
- Report benchmark commands, before/after measurements, test results and remaining
  limitations. Terra reviews the final changes; Luna addresses substantial
  implementation findings. Small review fixes may be handled by the parent.

No hardware movement, arming or physical qualification is authorized.

## Terra planning findings

- Cold range fitting transforms all mesh vertices at 243 poses and two jaw
  positions. Prefer the existing local bounding-box corner path: eight corners
  per visual conservatively enclose each sampled mesh without transforming every
  facet. Preserve the sampled-envelope limitation.
- Geometry cache keys must include all joints, jaw state and detail level. Image
  cache keys additionally include camera, projection, viewport, palette and device
  pixel ratio. TCP, ghosts and targets need appropriate overlay/fit invalidation.
- Preserve the shared rotated X/Z screen projection and rotated Y depth. Verify
  winding and open surfaces before culling. Inspect depth-bin ordering and path
  fill artifacts at overlapping parts.
- Current grouping omits blue and alpha. Use material identity plus shade bin or
  the complete color, and precompute shading colors outside the triangle loop.
- Coalescing updates alone does not make a long synchronous paint responsive;
  measure event-loop latency as well as frame times.
