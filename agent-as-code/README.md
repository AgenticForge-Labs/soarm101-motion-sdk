# Agent-as-code support files

This directory contains small experiment-local support files for external coding agents that
operate an already configured SO-ARM101 through the public `soarm101` CLI.

The Motion SDK does not define a model, provider, launcher, or sandbox. The bounded technical
tool contract is documented in [`docs/agent-arm101-cli.md`](../docs/agent-arm101-cli.md).

For experiments that intentionally provide task guidance, an opt-in robot/camera skill is
available at [`skills/robot-camera/SKILL.md`](skills/robot-camera/SKILL.md). It is not part
of the neutral SDK contract and should only be supplied when the experiment is meant to use
that strategy/completion policy.

## Hardware setup is shared

Robot ports, calibration-file references, and named camera settings are machine-local state in:

```text
~/.config/soarm101/workstation.json
```

The GUI and CLI use the same file. Do not copy physical device paths into experiment files.

Inspect it with:

```bash
soarm101 workstation show --json
soarm101 camera list --json
```

The Camera tab can assign stable names such as `overhead` and `wrist`. On Linux, camera
discovery prefers stable `/dev/v4l/by-id/*-video-index0` identifiers when available.

## Experiment setup

Copy the example:

```bash
cp agent-as-code/setup.example.json agent-as-code/setup.local.json
```

`camera_names` is optional experiment selection, not hardware configuration:

```json
{
  "camera_names": ["overhead", "wrist"]
}
```

An empty list means every camera currently configured in the workstation profile.

Validate names without opening camera hardware:

```bash
python agent-as-code/capture_observation.py \
  --setup agent-as-code/setup.local.json \
  --check
```

## Multi-camera capture helper

The helper calls the public CLI once per selected camera and writes one manifest:

```bash
python agent-as-code/capture_observation.py \
  --setup agent-as-code/setup.local.json \
  --label test
```

It is not a second camera implementation. An external reasoning agent should normally use the bounded camera surface directly:

```bash
soarm101 agent cameras
soarm101 agent capture overhead
soarm101 agent capture wrist
```

The unrestricted `soarm101 camera ...` commands remain available to human/developer
workflows.

## Agent runs

Agent launch configuration intentionally lives outside this SDK. A run may use Codex, Hermes,
another coding agent, or a custom harness. The SDK-side inputs can be limited to:

- the task/goal supplied by the experiment;
- the public `soarm101` executable;
- the neutral CLI contract in `docs/agent-arm101-cli.md`; and
- the shared workstation profile exposed through the CLI.

Before bounded physical motion, a human runs `soarm101 agent arm` interactively. The
agent then uses only `soarm101 agent ...` commands; it does not arm, disarm, relax, import
the Python SDK, or call unrestricted motion commands.

For runs intended to measure an agent's own task strategy, omit the optional task-facing
skill and provide only the bounded CLI contract/capabilities.

The GUI and CLI cannot own the same follower serial port simultaneously. A CLI capture also
cannot open a physical camera while the GUI already owns that same device.
