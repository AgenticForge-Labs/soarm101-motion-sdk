# Programs and saved positions

The simplest deterministic automation workflow in the GUI is:

```text
teach positions → arrange program steps → save → run
```

A Program is stored internally as a `MotionSequence` and executes through the same
`SequenceRunner` used by the Python SDK and CLI. The GUI name is deliberately simpler;
there is no separate Blockly-style runtime and no second path around the SDK's motion,
calibration, provenance, or safety checks.

## 1. Save useful positions

Open **Teach / Record** and choose the teaching source:

- **Follower** captures the follower's fresh measured pose.
- **Leader** captures the leader's fresh measured pose and binds it for follower replay.

Give each position a practical name. For a pick-and-place task, useful positions might be:

```text
above_pick
pick
above_drop
drop
park
```

The kinematic card shows the selected teaching source as the solid arm. Selecting an
existing saved position overlays it as a ghost so the current and stored poses can be
compared before replay.

Use **Move follower here** to validate a saved position only after the relevant physical
motion gates have already passed. A saved pose is calibration-bound on physical hardware.

## 2. Build a Program

Open **Programs**. The default **Position steps** tab contains the common actions.

### Move to a saved position

Choose:

- a saved position;
- **Joint / angular** or **Cartesian linear** movement; and
- a per-Move speed multiplier.

Then click **+ Move to position**.

The per-Move multiplier is combined with the Program's **Overall speed** at execution.
The normal SDK ceilings and guards remain authoritative; a multiplier cannot make an
otherwise invalid move acceptable.

Home and Rest can also be inserted as Move steps.

### Operate the gripper

Use:

- **+ Close gripper** for normalized position 0;
- **+ Open gripper** for normalized position 1; or
- **+ Custom gripper** for an intermediate normalized position.

Program execution uses the shared GUI gripper-speed preset. Home/Rest steps restore their
stored gripper position after the arm reaches the saved pose.

### Wait

Set a duration and click **+ Wait**. Waits are useful after opening/closing, for settling,
or for coordination with a human or external process.

## 3. Arrange the linear list

The Program list is intentionally simpler than a Blockly canvas. It executes from top to
bottom and reads directly as a procedure, for example:

```text
01  MOVE  above_pick · joint
02  MOVE  pick · joint · 0.50×
03  GRIPPER  CLOSE
04  WAIT  0.25 s
05  MOVE  above_pick · joint
06  MOVE  above_drop · joint
07  MOVE  drop · linear · 0.50×
08  GRIPPER  OPEN
09  MOVE  above_drop · joint
```

Use **Move up**, **Move down**, and **Delete step** to edit the list. Double-clicking a
step runs only that selected step when motion is available. The right-side kinematic card
shows the live follower as the solid arm and previews the selected Move destination as a
ghost.

## 4. Save and run

Give the Program a name and use **Save / replace program**. Saved Programs retain the same
calibration provenance requirements as MotionSequences.

Execution controls provide:

- **Run selected step**;
- **Run full program**;
- Repeat count;
- Overall speed;
- shared gripper speed;
- pause after the current step; and
- **STOP / HOLD**.

Validate a new physical Program one step at a time at conservative speed before running
the complete list.

## Recorded motion is optional

The **Trajectories / primitives** tab can insert a recorded trajectory or semantic motion
primitive into the same Program. Trajectory recording, replay, and editing are first-class
workflows in their own right; inserting one into a Program is simply another way to reuse
that motion. Recorded demonstrations are also the intended motion-data source for future
Robo Puppeteer integration.

For ordinary pick/place, loading, unloading, inspection, and bench automation, prefer
saved positions plus explicit gripper/wait actions when the important information is the
set of destinations rather than the exact continuous path. Recorded trajectories remain
first-class when the demonstrated path, timing, or future Robo Puppeteer motion data is
important.

## Radial / shoulder-pan patterns

A common SO-ARM101 pattern is to keep the arm's reach/elevation/wrist configuration fixed
while rotating around the base with `shoulder_pan`.

In **Programs → Position steps**:

1. Teach/select one saved base position.
2. Set start and end shoulder-pan **offsets** relative to that saved position.
3. Set the angular increment.
4. Click **Append pan pattern**.

The GUI expands the pattern into ordinary Program Move rows. Each row references the same
saved base pose and overrides only `shoulder_pan`; shoulder lift, elbow flex, wrist flex,
and wrist roll inherit the saved joint values. The gripper is not commanded by a Pan row,
so add an explicit gripper action when needed. The base saved pose itself is not modified
and no duplicate poses are created.

For example, a base pose at shoulder pan 10° with offsets -30°, 0°, and +30° produces
shoulder-pan targets -20°, 10°, and 40°. Generated targets are checked against the active
or configured shoulder-pan limits before the rows are appended.

These radial steps intentionally use joint/angular motion. A joint-overridden saved pose
cannot be replayed as a Cartesian-linear step because its stored TCP belongs to the
unmodified saved joint configuration.

## Kinematic previews

The shared kinematic view is diagnostic and educational:

- **Manual** — live follower.
- **Teleoperation** — live follower plus leader ghost.
- **Teach / Record** — selected teaching source plus saved-position ghost.
- **Edit recordings** — live follower plus scrubbed recorded-pose ghost.
- **Programs** — live follower plus selected destination ghost.

These previews use the SDK kinematic model. They do not replace physical validation,
collision checking, or the normal motion safety stack.
