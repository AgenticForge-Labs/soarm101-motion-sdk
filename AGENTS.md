# Coding Agent Instructions

1. Read `README.md`, `PLAN.md`, `docs/architecture.md`, and `docs/safety.md` first.
2. Keep all LeRobot imports inside the LeRobot backend.
3. Keep camera, video, tracking, OBS, and show concepts out of this repository.
4. Never add motion without explicit validation and tests.
5. Never add a placeholder API that reports success without performing documented behavior.
6. Add mock-backend tests before hardware tests.
7. Preserve LeRobot calibration compatibility.
8. Document units and coordinate-frame assumptions.
9. Check off plan items only after implementation and tests pass.
10. Preserve third-party attribution for adapted code.
11. Keep the core API independent from xArm-specific integer return codes.
