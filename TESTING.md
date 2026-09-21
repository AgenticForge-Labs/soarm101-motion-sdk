# Testing roadmap

This file is the handoff checklist for physical testing. Implementation can continue in
simulation before any of these steps are run. Work through the sections in order when
you are ready to put the real SO-ARM101 on the bench.

## Batch 1 — Setup, follower control, and leader readout

### 1. Launch and simulation smoke test

1. Pull the feature branch and install the GUI extra:
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
3. Start the live calibration sweep.
4. Move every arm joint and the gripper repeatedly through their complete safe travel.
   Gently touch both printed mechanical stops several times; do not hold or force a
   joint against a stop.
5. Let the recording finish. The SDK should derive each zero from the midpoint of the
   observed extrema, write symmetric limits, read them back, and save the calibration.
6. Reconnect normally (without "allow uncalibrated").
7. Confirm the GUI joint sliders now use the calibrated limits.

### 4. First follower motion validation — DO LATER

Do not start with Home/Rest or Cartesian moves.

1. Start near the mechanical midpoint with no payload.
2. Enable torque and verify the arm holds the position where it was enabled.
3. Test one joint at a time with approximately ±5° motion at low speed.
4. Verify the physical direction matches the GUI direction.
5. Press STOP/HOLD during a slow move and confirm the arm stops and holds.
6. Relax and confirm torque is disabled.
7. Only after all five joints pass should you test saved Home/Rest positions.
8. Only after joint motion passes should you test 5 mm Cartesian jogs.

### 5. Leader readout — DO LATER

1. Connect the follower and leader on separate USB serial adapters.
2. In Teach, select the leader port and a distinct leader robot ID/calibration.
3. Connect the leader with torque OFF.
4. Move each leader joint by hand and confirm the displayed angles and gripper position
   update independently of the follower.
5. Switch Teaching source between Follower and Leader and confirm the current-source
   readout follows the selected arm.
6. Do not enable live teleoperation yet. That is a later implementation/testing stage.

## Not implemented in Batch 1

- Point teaching and joint/linear replay from taught points.
- 50 Hz raw trajectory recording/playback.
- Timeline editing/cropping.
- Sequence editor and runner.
- Live leader-to-follower teleoperation.

Those will receive their own test sections when implemented.

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

## Still not implemented

- Sequence editor / Run tab combining points, gripper actions, waits, and trajectories.
- Live leader-to-follower teleoperation.
- Advanced trajectory editing (smoothing, splicing, keyframes, holds, loops/markers).
