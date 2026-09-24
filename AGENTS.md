# Coding agent instructions

1. Read `README.md`, `PLAN.md`, `docs/architecture.md`, and `docs/safety.md` first.
2. Keep ROS, camera capture, tracking, OBS, and show orchestration out of this repository.
3. Keep LeRobot optional and reference-only; do not import it from runtime code.
4. Treat the arm as five pose joints plus tool actuators.
5. Never add a motion method that reports success without executing documented behavior.
6. Validate complete Cartesian paths before motor commands.
7. Add simulator tests and fake-transport tests before physical hardware tests.
8. Preserve calibration and third-party attribution compatibility.
9. Do not claim physical validation until a real-arm smoke test passes.
10. Treat calibration IDs as motion provenance; physical replay must fail closed on missing or mismatched provenance.
11. Keep GUI and CLI features on shared SDK operations and saved libraries; the GUI must not shell out to CLI subprocesses for robot behavior.
12. When user-visible behavior, UI names, defaults, calibration rules, or safety gates change, update `README.md`, `CHANGELOG.md`, `TESTING.md`, and the relevant `docs/` files in the same change.
