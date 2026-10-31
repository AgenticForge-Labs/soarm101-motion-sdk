# Simulation

`SimulationBackend` is deterministic and hardware-free. It applies streamed commands immediately, so tests exercise the real motion profile, safety checks, FK, IK, and tool API.

CLI commands using `--simulation` use this deterministic, non-real-time backend. Use
`sim-demo --realtime` when wall-clock pacing is specifically needed; real-time
motion retains the controller's command lateness guard.

`PyBulletSimulationBackend` adds an optional visual model. It uses the same backend contract and planner; PyBullet is only a viewer/state mirror, not a separate motion implementation.

```bash
soarm101 sim-demo
soarm101 sim-demo --gui --realtime
```
