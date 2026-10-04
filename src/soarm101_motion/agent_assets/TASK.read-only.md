Validate the isolated SO-ARM101 agent environment without commanding physical motion.

1. Read SKILL.md completely.
2. Use robotctl.py to inspect capabilities and current state.
3. List or infer which named cameras are actually configured from capabilities.
4. Capture each configured allowed camera that is useful for observation.
5. Use vision_analyze on every fresh captured image and briefly describe what is visible.
6. Do not request any saved-pose motion, joint motion, Cartesian jog, gripper action, Sleep,
   or STOP/HOLD action. This run is intentionally read-only.
7. Report the observed robot state, available bounded capabilities, camera names, and local
   paths of any fresh images you analyzed.
