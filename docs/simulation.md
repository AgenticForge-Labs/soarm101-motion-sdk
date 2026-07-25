# Simulation

`SimulationBackend` is deterministic and hardware-free. It applies streamed commands immediately, so tests exercise the real motion profile, safety checks, FK, IK, and tool API.

`PyBulletSimulationBackend` adds an optional visual model. It uses the same backend contract and planner; PyBullet is only a viewer/state mirror, not a separate motion implementation.

```bash
soarm101 sim-demo
soarm101 sim-demo --gui --realtime
```
