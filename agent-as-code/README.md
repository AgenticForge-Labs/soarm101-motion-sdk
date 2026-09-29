# Agent-as-code support files

This directory contains small experiment-local support files for external coding agents that
operate an already configured SO-ARM101 through the public `soarm101` CLI.

The Motion SDK does not define an agent policy, reasoning loop, model, provider, launcher, or
sandbox. The generic tool contract is documented in
[`docs/agent-arm101-cli.md`](../docs/agent-arm101-cli.md).

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

It is not a second camera implementation. An agent may instead call the CLI directly:

```bash
soarm101 camera capture --name overhead --json
soarm101 camera capture --name wrist --json
soarm101 camera capture --all --json
```

## Agent runs

Agent launch configuration intentionally lives outside this SDK. A run may use Codex, Hermes,
another coding agent, or a custom harness. The SDK-side inputs can be limited to:

- the task/goal supplied by the experiment;
- the public `soarm101` executable;
- the neutral CLI contract in `docs/agent-arm101-cli.md`; and
- the shared workstation profile exposed through the CLI.

For runs intended to measure an agent's own task strategy, do not add an SDK-provided
problem-solving policy to those inputs.

The GUI and CLI cannot own the same follower serial port simultaneously. A CLI capture also
cannot open a physical camera while the GUI already owns that same device.
