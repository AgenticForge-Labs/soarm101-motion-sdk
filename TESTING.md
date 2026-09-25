# Testing roadmap

This file is the handoff checklist for physical testing. Implementation can continue in
simulation before any of these steps are run. Work through the sections in order when
you are ready to put the real SO-ARM101 on the bench.

## First physical bench session — stop gates

Use `docs/physical-run.md` as the short bench card. For the first powered session, stop
after any failed gate rather than continuing into later capabilities.

1. Discover/identify the follower and verify voltage, six motor IDs/models, diagnostics,
   and clean status with torque OFF.
2. Calibrate the follower through complete mechanical-stop sweeps. Record its displayed
   calibration ID and verify both the current calibration file and immutable history copy
   exist.
3. Disconnect the setup session and reconnect normally. A factory-range/uncalibrated
   session must now refuse torque enable before any goal or torque write.
4. Run the CLI 2° smoke test on **one joint at a time**, beginning with shoulder_pan.
   Confirm physical sign, clean return, STOP/power accessibility, and no fault/effort trip
   before proceeding to the next joint.
5. After all five pose joints pass, test the gripper from near mid travel with small
   normalized changes around 0.5. Do not make the first gripper command full open/close.
6. Characterize unloaded effort/current at the already-validated slow motions.
7. Only then save and test Home/Rest. Home/Rest replay is calibration-bound and now moves
   the arm first, then commands the saved gripper position.
8. Do **not** test Cartesian motion, recorded-trajectory replay, sequences, primitives, or
   leader→follower teleoperation during the first foundational run unless every preceding
   gate has passed.


## Batch 1 — Setup, follower control, and leader readout

### 1. Launch and simulation smoke test

1. Pull the latest tested branch/main and install the GUI extra:
   `pip install -e ".[gui]"`
2. Run:
   `soarm101-gui --simulation`
3. Confirm the window opens with Setup, Manual, Teleoperation, Record / Teach, Edit recordings, Run, and Log tabs.
4. Connect the follower simulation.
5. Enable it, jog joints/Cartesian axes, move the gripper, and confirm STOP/HOLD and
   Relax still work. Confirm Manual has only **Joint / angular** and **Cartesian** arm
   mode tabs and that the gripper tool panel remains visible in both.
6. Change the gripper speed preset in Manual and confirm the same preset appears in
   Teleoperation, Edit recordings, and Run.
7. Save the current simulated follower position as Home and Rest.
8. Move away, use Go Home / Go Rest, and confirm the controls target the saved poses.
9. Close and reopen the GUI and confirm Home and Rest are still present.

### 2. Physical follower connection — DO LATER

Before power-on, remove payloads, clear the workspace, secure the base, and keep the
physical power switch/plug immediately accessible.

1. Connect only the follower USB controller.
2. Start `soarm101-gui` and choose the follower serial port.
3. If the arm has never been calibrated, enable the explicit setup/uncalibrated
   connection option. Do not enable torque.
4. Confirm all six motors are detected and the displayed measured positions change
   sensibly when the unpowered arm is moved.

### 3. Mechanical-stop calibration — follower and leader — DO LATER

Follower and leader use the same calibration algorithm and the same Setup panel. Their
calibration files remain separate because each physical arm has its own robot ID and
measured endpoints.

1. Keep both arms torque OFF.
2. Open **Setup > Mechanical-stop calibration** and select **Follower**.
3. In **Setup / Calibrate**, click **Find Arms**, select **Follower**, then click
   **Connect for calibration (torque off)**. This sets the uncalibrated setup option
   for the follower connection. Do not press Enable.
4. Start recording. Six circular gauges reset to 0%. Each gauge fills while the
   encoder moves across its range, resets for the return traversal, and shows
   **DONE** with a brief black pop after two complete end-to-end traversals.
5. Gently move every joint and the gripper from one printed stop to the other and
   back. Do not hold or force a joint against a stop. Allow time for all six
   gauges to reach 2/2.
6. The minimum observed range is 2048 ticks (180°) for each pose joint and
   900 ticks for the gripper. The gripper gauge uses the selected arm's saved
   range as its visual scale when available; the leader and follower need not
   have the same gripper travel. **DONE** means two traversals were observed,
   not that the calibration file has been saved.
7. Recording ends automatically when all six reach 2/2; otherwise the selected
   time limit applies. Wait for **CALIBRATION SAVED** or **CALIBRATION FAILED**.
   Calibration must fail rather than save if any actuator is
   below its current minimum or has fewer than two traversals. A servo input-voltage
   fault also stops the sweep regardless of gauge progress. Otherwise the SDK derives
   each zero from the midpoint of
   the observed extrema, writes symmetric limits, reads them back, and saves calibration
   under the follower robot ID.
8. Record the short calibration ID shown in Setup. Verify
   `~/.config/soarm101/calibration/<follower-id>.json` exists and that an immutable
   fingerprint-named copy exists under
   `~/.config/soarm101/calibration/history/<follower-id>/`.
9. Reconnect the follower normally and confirm its GUI joint sliders use the calibrated
   limits.
10. Select **Leader** in the same Setup panel, then click
    **Connect for calibration (torque off)**. The discovered leader port and its own
    robot ID are used, and torque remains off.
11. In the same Setup calibration panel select **Leader** and repeat the exact sweep
    procedure. The same six gauges and thresholds are used, but the result is saved under
    the leader robot ID.
12. Record the leader calibration ID/history copy, reconnect the leader normally, keep
   its torque OFF, and confirm its readout changes
    sensibly as it is moved by hand.

### 4. First follower motion validation — DO LATER

Do not start with Home/Rest or Cartesian moves.

1. Start near the mechanical midpoint with no payload.
2. Before torque-on, the SDK now reads every selected motor's Present_Position and
   EEPROM Min/Max Position Limits. If any measured position is outside its active
   range, enabling must fail before any goal or torque-enable write. With torque off,
   manually move that joint back inside its calibrated range and investigate the
   calibration if the reported limits are unexpected.
3. Confirm that an intentionally uncalibrated/factory-range setup session refuses torque
   enable. This rejection must occur before any Goal_Position, Lock, or Torque_Enable write.
4. For the calibrated follower, use the CLI smoke test first:
   `soarm101 smoke-test --port PORT --robot-id ROBOT --joint shoulder_pan`.
   Its default motion is only 2° at 0.05 rad/s with 0.20 rad/s² acceleration, then it
   returns and relaxes.
5. Repeat the same 2° CLI smoke test for shoulder_lift, elbow_flex, wrist_flex, and
   wrist_roll one at a time. Verify the **physical positive direction** for each joint
   agrees with the GUI/model convention before any Cartesian testing.
6. After the CLI tests pass, enable in the GUI and make similarly small one-joint moves.
   Press STOP/HOLD during one slow move and confirm the arm stops/holds; Relax must then
   disable torque.
7. Put the relaxed gripper near mid travel by hand. Enable and command small normalized
   changes such as 0.50 → 0.55 → 0.45 → 0.50. Confirm direction and effort behavior.
   Do not use Open/Close endpoints as the first powered gripper test.
8. Only after all five joints and the small gripper test pass should you save fresh
   Home/Rest positions under the current calibration and replay them at low speed.
9. Only after joint-direction, STOP, gripper, Home/Rest, and FK checks pass should you test
   the default 2 mm Cartesian jogs. Start in **World / base** frame with a clear workspace.
10. Test one axis at a time in this order: X−/X+, Y−/Y+, Z−/Z+. After each click, verify
    both the orange target marker in the Manual joint-center view and the requested TCP diagnostic
    move only along the intended world axis before judging the physical arm motion.
11. If the model target is correct but the physical arm moves along the wrong axis, stop
    Cartesian testing and treat it as a joint-direction/calibration-to-model mismatch.
    Capture the GUI session log plus the five one-joint positive-direction observations.
12. After all six directions agree, click the same jog button three times rapidly. Confirm
    the queue reports two waiting commands after the first starts, executes all three
    sequentially, and STOP/HOLD immediately cancels the active jog and clears the remainder.

### 4a. Motor effort/current characterization — DO LATER

Do this after the first small joint-motion checks and before relying on the effort guard
for larger or continuous motion. Raw current/load are safety indicators, not calibrated
force.

1. Open Setup > **Motor effort safety / characterization**. Confirm the guard is enabled
   and note the session defaults and each motor's effective thresholds.
2. With torque OFF and no payload, press **Refresh readings** once. Confirm all six motors
   return plausible raw values and there are no communication errors.
3. Press **Reset peaks**, place the arm near its centered pose, enable torque, and let it
   hold without commanded motion for several seconds. Record the displayed per-motor
   current/load values and peaks.
4. Run the already-validated slow ±5° motion on one joint at a time. After each move,
   inspect the session peaks. The backend samples effort during guarded motion, so the
   peak display should capture values seen by the safety monitor without adding a second
   serial-read loop from the GUI.
5. Keep the default thresholds for the first characterization pass. If normal unloaded
   holding/motion produces false trips, **Relax first**, then make a small explicit
   threshold adjustment and repeat the same test. Do not change effort settings while
   torque is enabled.
6. If a trip occurs, confirm the global status and effort panel show the latched motor and
   reason, and that motion remains blocked. Remove any contact/obstruction before pressing
   **Clear latched trip**. Clearing the latch must not itself move the arm.
7. Test **Reset peaks** between controlled conditions so unloaded hold, single-joint
   motion, gripper motion, and later payload tests can be compared separately.
8. Treat disabling the effort guard as diagnostic-only. The GUI requires explicit
   confirmation, the setting lasts only for the current connection/session, and all
   other SDK safety layers remain active.
9. Record useful provisional per-joint ranges for current and |load|. Only after repeated
   tests should the defaults or per-motor overrides be tightened/relaxed in code.

### 5. Leader readout — DO LATER

1. Connect the already-calibrated follower and leader on separate USB serial adapters.
2. In Setup, select the leader port and its distinct leader robot ID.
3. Connect the leader with torque OFF. If it still needs calibration, use the shared Setup calibration workflow before returning to teaching.
4. Move each leader joint by hand and confirm the displayed angles and gripper position
   update independently of the follower.
5. In Record / Teach, switch the teaching source between Follower and Leader and confirm the current-source readout follows the selected arm.
6. Do not enable live teleoperation yet. That is a later implementation/testing stage.

## Implemented after Batch 1

Point teaching, trajectory recording/replay, timeline editing, sequence execution,
guarded live teleoperation, and advanced trajectory editing are now implemented in
software. Their deferred checks are below.

## Batch 2 — Taught points, trajectory recording, and trajectory editing

### Calibration-provenance replay gate — before physical artifact replay

1. Save a fresh follower taught point and confirm its pose JSON contains
   `source_calibration_id` and `target_calibration_id` matching the active follower.
2. Record one leader trajectory only after both arms are calibrated. Its metadata should
   retain the leader source calibration and the follower target calibration.
3. Derived trajectory edits must preserve the source provenance and target binding.
4. Do not reuse old pre-fingerprint physical artifacts. They are expected to fail closed.
5. After any future follower recalibration, existing poses/trajectories/sequences are
   expected to refuse physical replay until explicitly reviewed and re-saved/re-bound.
6. Simulation intentionally ignores this physical provenance gate.


These checks can wait until the Batch 1 hardware checks pass.

### 6. Software-only point teaching

1. Run `soarm101-gui --simulation`.
2. Connect and enable the follower simulation.
3. In Record / Teach, leave Follower selected as the teaching source.
4. Move the simulated follower to a non-home joint pose.
5. Save it as a named taught point such as `test_point_1`.
6. Move away from the point.
7. Replay it in Joint / angular mode and confirm the measured joint state returns to it.
8. Move away again, replay it in Cartesian linear mode, and confirm the TCP returns to
   the saved pose.
9. Restart the GUI and confirm the taught point remains in the pose library.

### 7. Software-only raw trajectory recording and replay

1. Connect the follower simulation and the leader simulation.
2. Select Leader as the teaching source.
3. Enter a unique raw trajectory name such as `sim_wave_raw_01`.
4. Leave effort/current recording off for the first test.
5. Start recording, allow it to run for several seconds, then stop.
6. Confirm the Edit recordings tab opens with a six-lane timeline (five joints + gripper).
7. Confirm the raw file appears under the library as `raw`.
8. Try to reuse the same raw name and confirm the GUI refuses to overwrite it.
9. Enable the follower simulation and replay the raw trajectory.
10. Confirm replay first moves safely to the clip start and then executes the recorded
    stream.

### 8. Software-only trajectory editing

1. Load the raw test trajectory.
2. Scrub through the timeline and verify the cursor time updates.
3. Set selection start/end inside the full recording.
4. Replay only the selection.
5. Set speed scale to 0.5x and replay the selection; duration should approximately double.
6. Save the selection as a new edited name such as `sim_wave_trim_v1`.
7. Confirm both the raw and edited versions remain in the library.
8. Reload the raw version and confirm it was not changed by the edit.
9. Load the edited version and replay it.
10. Try a faster speed scale. If the resulting trajectory exceeds configured joint
    speed, acceleration, or command-step limits, replay should be rejected rather than
    silently retimed.

### 9. Physical taught-point replay — DO LATER

Only continue after Batch 1 first-motion validation passes.

1. Teach one point from the follower while torque is off, then enable torque.
2. Move a small distance away and replay the point in Joint mode at low speed.
3. Verify physical direction and endpoint before trying a larger move.
4. Teach a second point with at least 20 mm of clear workspace around the whole path.
5. Replay it in Cartesian linear mode at low speed.
6. Keep a hand at the physical power switch throughout the first tests.

### 10. Physical leader recording — DO LATER

1. Keep leader torque OFF and follower disconnected or relaxed for the first capture.
2. Record a slow 3–5 second leader motion at 50 Hz with effort/current recording OFF.
3. Confirm the reported median sample rate is close to 50 Hz and that no capture errors
   are logged.
4. Inspect/crop the recording before any follower replay.
5. Put the follower near the recorded start area, enable it, set playback speed to 0.5x,
   and replay with the workspace clear.
6. Confirm the pre-roll is smooth and the replay has no discontinuous jump into sample 1.
7. Press STOP/HOLD during a slow replay and confirm cancellation/hold works.
8. Only after basic recording is stable should you enable effort/current recording.
   Compare achieved sample rate and logs; disable the diagnostic channel if serial load
   makes 50 Hz capture unreliable.

### 11. Physical edited-motion replay — DO LATER

1. Crop a known-safe physical recording to a shorter region and save it under a new name.
2. Replay at 0.5x first.
3. Verify the crop boundary does not create a joint jump; the SDK should reject it if it
   violates command-step/speed/acceleration limits.
4. Test 1.0x only after the slower replay passes.
5. Treat >1.0x playback as a separate validation: faster timing is allowed only when all
   configured hard limits still pass.

## Batch 3 — Sequences, live teleoperation, and advanced motion primitives

### 12. Software-only sequence editor and runner

1. Run `soarm101-gui --simulation`, connect the follower simulation, and enable it.
2. Ensure Home/Rest and at least one taught point exist. Keep one saved trajectory from
   Batch 2 if available.
3. Open Run and create a short sequence such as:
   Home → Point → Gripper 0.3 → Wait 0.25 s → Home.
4. Save the sequence, clear/load it again, and confirm step order is preserved.
5. Use **Run selected step** on each step individually.
6. Run the entire sequence once at 0.5× speed, then at 1.0×.
7. Set Repeat to 2 and confirm the complete sequence runs twice.
8. Add a WAIT of at least 1 second. Start the sequence, press
   **Pause after current step**, and confirm it pauses at a step boundary or pauses the
   WAIT timer. Resume and confirm execution continues.
9. During another run press **STOP / HOLD** and confirm the active sequence is cancelled
   and the follower holds.
10. Restart the GUI and confirm saved sequences remain available.

### 13. Software-only advanced trajectory editing and primitives

1. Load a known trajectory in Edit recordings.
2. Apply a 5-sample smoothing window. Confirm the displayed shape changes while the
   first and last poses remain unchanged.
3. Select an interior region and use Delete. Save As a new edited trajectory.
4. Reload the source trajectory to confirm it was not modified.
5. Insert a 0.5 second hold at the scrub cursor and confirm duration increases by about
   0.5 seconds.
6. Set a small joint keyframe at the cursor, or set a gripper keyframe within [0, 1].
   Replay validation remains authoritative; deliberately extreme edits should be rejected.
7. Add markers such as `contact`, `beat`, or `release` and confirm they appear on
   the timeline.
8. Select a region and make a 2× repeated clip. Inspect the loop boundary before replay;
   a discontinuous loop must be rejected by normal motion safety validation.
9. Save a safe edited trajectory, give it a semantic name such as `wave_gentle`, add
   tags, and promote it to a motion primitive.
10. In Run, add that primitive as a sequence step and execute it in simulation.

### 14. Software-only live teleoperation lifecycle

The automated tests exercise moving streaming targets directly. The GUI simulation is
mainly a lifecycle/UI check because the simulated leader has no physical hand input.

1. Connect follower simulation and leader simulation.
2. Leave the follower connected with torque off; starting teleoperation should latch its measured pose, enable hold, and run the alignment step.
3. In Teleoperation choose **Relative / clutch-safe** and leave Mirror gripper enabled.
4. Confirm **20 Hz — default** is selected, then click **Align follower and start**. Confirm the status reports alignment before live following and ordinary follower jog/sequence controls are disabled while teleop is active.
5. Stop live teleoperation and confirm the follower returns to holding state.
6. Start it again and press the global **STOP / HOLD**. Confirm teleop ends.
7. Disconnect the leader while teleop is active. Confirm follower teleop terminates and
   holds rather than continuing with stale targets.
8. Repeat the lifecycle with **Absolute calibrated angles** as a software check; on hardware,
   Absolute mode is deferred until calibration/alignment validation below.
9. Stop teleoperation. Click **Park leader here** and confirm leader simulation reports
   PARKED / torque on; click **Release leader** and confirm FREE / torque off.
10. Put the simulated leader and follower at different poses. With gripper matching
    unchecked, use **Move leader → follower pose** and then the reverse direction; confirm
    each destination converges while the source remains unchanged.
11. Diverge the two simulations again and use **Relink here — no motion**. Confirm neither
    arm performs an alignment move before relative teleoperation begins.
12. While teleoperation is active, click **Stop → Manual + park leader**. Confirm the
    follower remains holding, Manual opens, and the leader reports PARKED.
13. Exercise Slow, Normal, and Fast gripper presets through a manual move, a sequence
    gripper step, and recorded-trajectory replay; inspect simulator/backend tests for the
    raw speed propagation because simulation itself has no motor-speed dynamics.

### 15. Physical sequence execution — DO LATER

Only continue after the Batch 1 and Batch 2 physical checks pass.

1. Build a sequence containing only Home, one already-validated small taught point, a
   short WAIT, and Home.
2. Set global sequence speed to 0.5× and Repeat to 1.
3. Keep the physical power switch accessible and execute each step with
   **Run selected step** before running the whole sequence.
4. Run the full sequence once.
5. Test **Pause after current step** and Resume.
6. Test **STOP / HOLD** during a slow joint step and during a WAIT.
7. Add a previously hardware-validated trajectory only after the point sequence passes.
8. Increase speed/repeat only after single-run behavior is repeatable.

### 16. Physical relative leader → follower teleoperation — DO LATER

This is a new continuous-control path and has not yet been physically validated.

1. Complete calibration, joint-direction, FK, low-speed joint, and STOP checks first.
2. Secure both bases, clear the follower workspace, remove payloads, and keep physical
   follower power immediately accessible.
3. Connect both arms. Keep the leader torque OFF and leave the follower torque OFF before starting teleoperation.
4. Before live teleoperation, validate parking by holding the leader near a safe supported
   pose, clicking **Park leader here**, and confirming it does not jump when torque enables.
   Click **Release leader** and confirm it becomes back-drivable again. Do not continue if
   parking causes unexpected motion.
5. With both arms already close to one another and the gripper-match box unchecked, test
   **Move leader → follower pose** at the conservative default synchronization speed, then
   test the reverse direction. Keep physical power accessible and validate each destination
   joint direction before increasing pose differences.
6. Deliberately leave the arms at slightly different safe poses and test **Relink here —
   no motion** in Relative mode. Confirm the follower only latches its own measured pose;
   there must be no alignment move before leader motion begins.
4. Put both arms in comfortable poses with the leader inside the follower's calibrated travel. Keep the leader still during startup and keep the follower in a clear nearby pose so the guarded alignment move is small.
5. Select **Relative / clutch-safe**, initially disable gripper mirroring, select
   **5 Hz — slow check**, and click **Align follower and start**. Confirm torque enable does not cause a jump and the guarded alignment completes (or reports a staged offset near a model limit) before live following begins.
6. Move only one leader joint a few degrees, slowly. Confirm the follower moves the same
   signed delta after alignment.
7. Watch the live follower-cycle and queued-age values. At 5 Hz the period is 200 ms;
   processing should remain comfortably below that and queued age should stay low rather
   than increasing over time.
8. Return that joint and repeat for the other four joints one at a time.
9. Test STOP/HOLD while making a slow motion. The follower must stop/hold and teleop must
   terminate.
10. Restart teleop, then disconnect/unplug the leader data connection. The follower must
    stop receiving stream targets and hold.
11. Re-enable gripper mirroring and test a small leader gripper delta.
12. After 5 Hz is repeatable, run the same checks at 10 Hz (100 ms period).
13. Validate 20 Hz (50 ms period) next even though it is the current software default, then treat 50 Hz (20 ms period) as a separate experimental stage. Do not increase merely because motion looks smooth; record cycle time, queued sample age, overruns, communication errors, STOP response, following errors, and effort trips as described in `docs/teleoperation.md`.
14. If follower processing exceeds the selected period repeatedly or queued sample age
    grows, the software should terminate teleop and hold. Reduce the rate before retrying.
15. Deliberately move the leader faster only enough to verify configured step/speed/
    acceleration guards reject unsafe streaming rather than following it.
16. Do not treat software STOP as an emergency stop; physical power remains the ultimate
    intervention during these tests.

### 17. Physical absolute teleoperation — DO LATER, AFTER RELATIVE PASSES

1. Verify leader and follower use compatible calibrated joint signs and zero conventions.
2. Place the follower in a clear pose reasonably near the leader to keep automatic alignment travel small, and keep the leader still during startup.
3. Select **Absolute calibrated angles** at 5 Hz and click **Align follower and start**. The start sequence should latch the follower before torque enable and perform a guarded alignment; if the leader pose is outside the follower's calibrated travel it must refuse before live following. If the initial difference exceeds command-step or
   other stream limits, rejection is expected and preferable to a jump.
4. Test only a few degrees on one joint at a time before coordinated motion.
5. Repeat the STOP and leader-readout-loss tests from Relative mode.

### 18. Physical advanced motion primitives — DO LATER

1. Start from a trajectory already proven safe on hardware.
2. Apply one edit at a time and save every version under a new name.
3. Replay smoothing/hold/keyframe/splice variants first at 0.5×.
4. Inspect repeated/looped clips especially carefully at the end→start boundary.
5. Promote only hardware-validated edited trajectories to show motion primitives.
6. Run a primitive first by itself, then as a single sequence step, then in a multi-step
   sequence.
7. Record which primitive versions, speed ranges, and payload conditions have actually
   passed hardware testing.

## Software-complete / hardware-validation pending

The teaching workflow is now implemented through the Run/primitive layer. Remaining work
is physical validation and future optional capabilities such as Cartesian velocity
streaming, richer mesh collision models, and show-level orchestration in the appropriate
Robo Puppeteer/Director repositories.
