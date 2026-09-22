# Testing roadmap

This file is the handoff checklist for physical testing. Implementation can continue in
simulation before any of these steps are run. Work through the sections in order when
you are ready to put the real SO-ARM101 on the bench.

## Batch 1 — Setup, follower control, and leader readout

### 1. Launch and simulation smoke test

1. Pull the latest tested branch/main and install the GUI extra:
   `pip install -e ".[gui]"`
2. Run:
   `soarm101-gui --simulation`
3. Confirm the window opens with Setup, Control, and Teach areas.
4. Connect the follower simulation.
5. Enable it, jog joints/Cartesian axes, move the gripper, and confirm STOP/HOLD and
   Relax still work.
6. Save the current simulated follower position as Home and Rest.
7. Move away, use Go Home / Go Rest, and confirm the controls target the saved poses.
8. Close and reopen the GUI and confirm Home and Rest are still present.

### 2. Physical follower connection — DO LATER

Before power-on, remove payloads, clear the workspace, secure the base, and keep the
physical power switch/plug immediately accessible.

1. Connect only the follower USB controller.
2. Start `soarm101-gui` and choose the follower serial port.
3. If the arm has never been calibrated, enable the explicit setup/uncalibrated
   connection option. Do not enable torque.
4. Confirm all six motors are detected and the displayed measured positions change
   sensibly when the unpowered arm is moved.

### 3. Mechanical-stop calibration — DO LATER

1. Keep follower torque OFF.
2. Open Setup > Calibration.
3. Start the live calibration sweep. Six circular gauges should reset to 0% and
   begin filling from the observed encoder extrema, not from elapsed time.
4. Move every arm joint and the gripper repeatedly through their complete safe travel.
   Gently touch both printed mechanical stops several times; do not hold or force a
   joint against a stop.
5. Confirm every pose-joint gauge reaches PASS. The current provisional minimum is
   2048 ticks (180°) for each of the five arm joints. The gripper uses a separate,
   deliberately conservative 256-tick minimum until its physical travel is characterized.
   The gauge also shows the raw observed tick span so these expectations can be refined.
6. Let the recording finish. Calibration must fail rather than save if any actuator is
   below its current minimum. Otherwise the SDK should derive each zero from the midpoint
   of the observed extrema, write symmetric limits, read them back, and save the calibration.
7. Reconnect normally (without "allow uncalibrated").
8. Confirm the GUI joint sliders now use the calibrated limits.

### 4. First follower motion validation — DO LATER

Do not start with Home/Rest or Cartesian moves.

1. Start near the mechanical midpoint with no payload.
2. Before torque-on, the SDK now reads every selected motor's Present_Position and
   EEPROM Min/Max Position Limits. If any measured position is outside its active
   range, enabling must fail before any goal or torque-enable write. With torque off,
   manually move that joint back inside its calibrated range and investigate the
   calibration if the reported limits are unexpected.
3. Enable torque and verify the arm holds the position where it was enabled.
4. Test one joint at a time with approximately ±5° motion at low speed.
5. Verify the physical direction matches the GUI direction.
6. Press STOP/HOLD during a slow move and confirm the arm stops and holds.
7. Relax and confirm torque is disabled.
8. Only after all five joints pass should you test saved Home/Rest positions.
9. Only after joint motion passes should you test 5 mm Cartesian jogs.

### 5. Leader readout — DO LATER

1. Connect the follower and leader on separate USB serial adapters.
2. In Teach, select the leader port and a distinct leader robot ID/calibration.
3. Connect the leader with torque OFF.
4. Move each leader joint by hand and confirm the displayed angles and gripper position
   update independently of the follower.
5. Switch Teaching source between Follower and Leader and confirm the current-source
   readout follows the selected arm.
6. Do not enable live teleoperation yet. That is a later implementation/testing stage.

## Implemented after Batch 1

Point teaching, trajectory recording/replay, timeline editing, sequence execution,
guarded live teleoperation, and advanced trajectory editing are now implemented in
software. Their deferred checks are below.

## Batch 2 — Taught points, trajectory recording, and trajectory editing

These checks can wait until the Batch 1 hardware checks pass.

### 6. Software-only point teaching

1. Run `soarm101-gui --simulation`.
2. Connect and enable the follower simulation.
3. In Teach, leave Follower selected as the teaching source.
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
6. Confirm the Trajectories tab opens with a six-lane timeline (five joints + gripper).
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

1. Load a known trajectory in Trajectories.
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
2. Enable only the follower simulation.
3. In Teach choose **Relative / clutch-safe** and leave Mirror gripper enabled.
4. Select 10 Hz and start live teleoperation. Confirm the UI reports the selected guarded
   stream rate and ordinary follower jog/sequence controls are disabled while teleop is active.
5. Stop live teleoperation and confirm the follower returns to holding state.
6. Start it again and press the global **STOP / HOLD**. Confirm teleop ends.
7. Disconnect the leader while teleop is active. Confirm follower teleop terminates and
   holds rather than continuing with stale targets.
8. Repeat the lifecycle with Absolute mode only as a software check; on hardware,
   Absolute mode is deferred until calibration/alignment validation below.

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
3. Connect both arms. Keep the leader torque OFF; enable the follower.
4. Put both arms in comfortable poses. They do not need identical poses for Relative mode.
5. Select **Relative / clutch-safe**, initially disable gripper mirroring, select
   **5 Hz — first hardware tests**, and start teleop.
6. Move only one leader joint a few degrees, slowly. Confirm the follower moves the same
   signed delta and does not jump when teleop starts.
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
13. Treat 20 Hz (50 ms period) and 50 Hz (20 ms period) as separate experimental
    validation stages. Do not increase merely because motion looks smooth; record cycle
    time, queued sample age, overruns, communication errors, STOP response, following
    errors, and effort trips as described in `docs/teleoperation.md`.
14. If follower processing exceeds the selected period repeatedly or queued sample age
    grows, the software should terminate teleop and hold. Reduce the rate before retrying.
15. Deliberately move the leader faster only enough to verify configured step/speed/
    acceleration guards reject unsafe streaming rather than following it.
16. Do not treat software STOP as an emergency stop; physical power remains the ultimate
    intervention during these tests.

### 17. Physical absolute teleoperation — DO LATER, AFTER RELATIVE PASSES

1. Verify leader and follower use compatible calibrated joint signs and zero conventions.
2. Manually place the follower very near the leader's measured joint pose before starting.
3. Start Absolute mode at low speed. If the initial difference exceeds command-step or
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
