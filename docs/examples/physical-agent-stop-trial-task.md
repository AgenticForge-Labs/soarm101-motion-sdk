# Attended SO-ARM101 agent motion/STOP experiment

This is a HUMAN-SUPERVISED test with the real, unloaded hobby arm.
Only use the broker capabilities that are advertised. Do not call
unavailable tools, use shell motor commands, rearm torque, or retry a
faulted motion. Never exceed the per-request 0.5° cap.

1. Read the current robot state and report the measured joint angles.
2. If the shoulder pan can move safely by +0.5° within calibrated
   limits and the human has approved the clear path, request one
   +0.5° shoulder-pan jog at no more than 2°/s.
3. Read back measured joint positions. If successful, request one
   -0.5° shoulder-pan jog and inspect state again.
4. Report actual before/after joint positions and any error. Do not
   perform any pose, gripper, Cartesian, Sleep or other movement.

A human may press Ctrl+C at any time. Never depend on this task
prompt as the safety STOP: the trusted host supervisor owns STOP,
which revokes authority and requires a new human session.
