Validate the isolated SO-ARM101 agent environment without commanding physical motion.

1. Read SKILL.md completely.
2. Use robotctl.py to inspect capabilities and current state.
3. Treat an absent motion-authority lease as expected for this validation. Do not stop merely
   because `authority.armed` is false: capabilities, state, and named-camera capture are
   intentionally read-only and remain available.
4. Read the configured camera names from capabilities and capture every configured camera using
   robotctl.py's default observation directory.
5. Inspect every fresh captured image using the agent-specific local image-inspection method
   described in SKILL.md. If this harness cannot inspect local images, say so explicitly rather
   than guessing, but still capture every configured camera.
6. Do not request any saved-pose motion, joint motion, Cartesian jog, gripper action, Sleep,
   or STOP/HOLD action. This run is intentionally read-only.
7. Report the observed robot state, available bounded capabilities, camera names, local paths
   of all fresh captures, and whether each image was actually inspected.
