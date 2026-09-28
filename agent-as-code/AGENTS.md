# Agent-as-code operating instructions

You are operating a physical SO-ARM101 through a constrained AgenticForge interface
inside an NVIDIA OpenShell sandbox.

Read `/opt/agent-as-code/CLI.md` before acting.

## Allowed physical interface

Use **only** `robotctl` for physical observation and robot action.

Do not attempt to find, open, or control serial ports, USB camera devices, servo
registers, motor buses, or alternative robot libraries. Those devices intentionally
exist only on the host side of the sandbox.

Start every task with:

```bash
robotctl health
robotctl observe --label initial
```

Use `robotctl state` when robot pose/state would reduce uncertainty.

## Task behavior

- Accept the supplied natural-language task as the goal. Do not add an unstated goal.
- Treat the camera marked primary in observation output as the canonical task view.
  Auxiliary cameras are additional evidence.
- Actually inspect fresh image files with your available vision capability; a pathname
  alone is not visual evidence.
- Observe again after meaningful physical changes instead of assuming an action worked.
- Prefer small, deliberate jogs while learning image-to-robot direction relationships.
- Respect host-executor limits. A rejected request is evidence to re-plan, not permission
  to bypass the executor or SDK.
- Never weaken or work around calibration, provenance, path, joint, following-error,
  fault, effort, or other SDK guards.
- Stop issuing physical actions once the task is complete or safe progress is no longer
  possible.
- Software stop/hold behavior is not a certified emergency stop; the human operator owns
  physical power intervention.

## Benchmark integrity

Do not modify these operating instructions, the benchmark task, the host executor,
Motion SDK source, calibration files, or sandbox policy during a scored run. Do not
install an alternative hardware-control path. Record failures and limitations rather
than silently repairing the benchmark.

The security boundary is OpenShell plus the host executor. Agent-specific internal
approval/sandbox prompts may be disabled so Codex and Hermes receive the same effective
physical permissions.
