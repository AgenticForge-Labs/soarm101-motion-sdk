# GUI mesh viewer and Live Manual

The persistent follower panel renders the complete official SO-ARM100
repository's SO-101 new-calibration visual meshes while settled. During direct
view dragging only, it switches to separately packaged topology-preserving
quadric-decimated LOD derivatives, then restores the complete meshes on release.
The source
revision is `385e8d7c68e24945df6c60d9bd68837a4b7411ae`; original project license
and mesh-to-link visual transforms are packaged under
`src/soarm101_motion/gui/assets/so101/`. The source is licensed under Apache-2.0.
The pinned source URDF is retained with the assets, and its 17 visual mappings
(link, mesh, material, XYZ and RPY) are checked directly against the manifest.
Blender 5.2.2's collapse-mode quadric decimator generated the drag LOD at ratio
0.05 (the reproducible script is `scripts/generate_gui_mesh_lod.py`); the
renderer draws every retained triangle in either level. Each visual is
transformed through `SO101KinematicModel.link_frames`; those SDK frames and
calibrated joint values remain authoritative. Jaw presentation geometry
continues to use the SDK's stock-gripper mapping.

The CPU renderer reads binary STL vertices with a strided NumPy view, reuses
immutable local mesh arrays, groups shaded facets by material and shade bin,
and caches the completed mesh image for an unchanged pose and view. The sampled
Fit Range envelope uses transformed local mesh bounding-box corners at the same
243 joint samples and both jaw endpoints. Those bounds conservatively contain
each full mesh; the sample grid remains an illustrative viewing envelope, not a
continuous-range guarantee or collision authority.

### CPU rendering profile

Run the software-only offscreen benchmark with
`.venv/bin/python scripts/benchmark_gui_mesh.py --size 480 --runs 7`. It uses no
robot connection or device. On an 11th Gen Intel Core i7-1185G7 (8 logical CPUs),
Python 3.14.4 and Qt 6.11.2, the 480 × 480 standalone viewport cold-show took
0.488 s. Each table cell gives median / slowest of 7 repetitions, except Qt
dispatch and heartbeat samples, whose observed event counts are listed below.
The standalone view and the real compact `RobotStatusPanel` layout are both
measured; with the panel at 340 × 900, its view laid out at 314 × 340.

| Workload | Standalone 480 × 480 | Sidebar view 314 × 340 |
| --- | ---: | ---: |
| Direct unchanged `render()` (synchronous comparison) | 4.32 / 11.01 ms | 3.70 / 4.36 ms |
| Continuous Qt mouse drag dispatch plus paint servicing | 34.48 / 39.52 ms | 30.74 / 31.69 ms |
| Repeated identical measured joints and jaw | 1.09 / 1.10 ms | 1.09 / 1.29 ms |
| Changing measured joints and jaw | 32.76 / 35.54 ms | 31.06 / 32.42 ms |
| Queued-update to paint dispatch latency (16 samples) | 0.06 / 0.15 ms | 0.04 / 0.07 ms |
| 10 ms Qt heartbeat lateness (15 samples) | 23.89 / 70.18 ms | 21.06 / 47.06 ms |

The direct `render()` row is retained only as an apples-to-apples synchronous
paint comparison; it does not represent UI interaction. Qt mouse values measure
move events during one continuous drag and event processing. The full-detail
repaint after releasing the mouse is not included. Public setter
cases call the measured joint and jaw setters, including repeated identical
feedback and changing pose values. Paint dispatch latency measures time from a
queued update request to the paint event reaching Qt's event filter; it is not
paint duration. Timer lateness reflects event-loop scheduling delays, including
synchronous paint stalls. These local, offscreen measurements do not predict
desktop interaction frame rate across CPUs or display backends. Synchronous
painting can block GUI events, including the GUI heartbeat and a user's STOP
click, even though robot processing runs in a worker. The range fit remains a
243-pose sample and may not bound every unsampled intermediate pose.

The LOD contains 3,939 triangles across the visual manifest versus 47,845
full-detail instances (8.2%); each displayed LOD facet is still a retained
triangle from the Blender quadric-decimated surface.

The viewer renders the official moving-jaw STL through the stock gripper's
`STOCK_JAW_JOINT` origin, axis, and nominal travel, mapped from normalized gripper
position (0 closed, 1 open). The same SDK link frames drive the arm meshes. The
viewer fits conservative transformed mesh bounds and SDK TCP across a deterministic
3-value-per-joint grid over the SDK joint ranges (243 poses). The sampled fit is
cached and remains stable across measured poses and aspect changes. It is a useful
viewing envelope, not a formal proof that every intermediate pose fits. The picture
is presentation-only; it does not establish
physical clearance, collision sensing, or a safe workspace. Drag to orbit, wheel to
zoom, and use Fit after changing the view.

Joint motion presets are Slow (10°/s, 100°/s²), Medium (50°/s, 500°/s²), and Fast
(100°/s, 1000°/s²). A session disables any preset outside its configured stream
envelope. Custom leaves numeric tuning available, and the SDK checks the selected
limits again when starting the stream.

Standard Side, Front, Top, and Isometric changes preserve the current effective
pixel scale and keep the world-space orbit pivot centered. Fit range and Fit pose
explicitly recompute framing and reset zoom; Auto fit continues to recompute while
enabled. Resizing retains the existing refit behavior.

Live Manual slider and numeric editing also supports the gripper. Startup reads
the gripper position through the active SDK tool owner and seeds the control and
contact latch from that measurement. A user edit streams the arm and gripper
together; the worker applies the teleoperation rate limit, contact stall latch,
safe closed inset, relief, and opening release. The selected gripper speed preset
is used for the guarded stream. Measured jaw feedback updates the visual and
readout. Separate gripper move buttons and speed selection are unavailable during
the run. Stop seeds controls from fresh measured state after confirmed HOLD.

Live Manual is off by default. The operator must connect and explicitly enable
torque/HOLD first. Enabling requires a connected ready follower holding torque and
no competing motion or recording. It starts one existing guarded joint stream,
whose controller captures and validates the measured joint baseline under its
state lock. The returned exact baseline initializes the follower, target controls,
and ready state without an earlier second joint read. The gripper is initialized
from measured state and receives no command until explicitly edited. Five-joint slider/numeric changes
replace one synchronized latest-target slot. A fixed-period host follower applies
speed, acceleration, and command-step bounds; each sample still passes the SDK's
workspace, effort, following-error, fault, and state checks. Gripper operations and
other arm actions are excluded while the stream is active.

Turning Live Manual off, STOP, disconnect, a stream/feedback error, or loss of the
GUI heartbeat requests SDK HOLD and ends the stream. Each stream result publishes
the SDK measured positions and derived TCP through the GUI; no competing servo bus
read is used during streaming. The post-HOLD measurement seeds ordinary joint
targets, and targets remain disabled until fresh feedback arrives if that read is
unavailable. The UI heartbeat runs every
500 ms; two seconds without it faults closed and requests HOLD. A quiet, settled
target may remain held while the UI remains present. Failed or unavailable HOLD
sets a persistent worker fault latch, keeps the GUI busy, reports **HOLD
UNCONFIRMED**, and rejects further motion and torque enable. Disconnect, shutdown,
close, polling, and reconnect do not clear the latch. Pressing STOP retries through
the existing arm STOP route; only a successful STOP followed by a connected state
with torque enabled clears the latch. It never resumes Live Manual automatically.
Rate controls are disabled during a Live Manual run; rate changes take effect only
after the stream is stopped and started again. Ordinary arm and gripper operations
are rejected while the worker owns the stream. Software HOLD is not a safety-rated
emergency stop. This change has software validation only and has not been physically
qualified.
