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
