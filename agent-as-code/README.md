# Agent-as-code support files

This directory contains machine-local support files for experiments that let an external
coding agent operate an already configured SO-ARM101 through the public `soarm101` CLI.

The Motion SDK does not define an agent policy, reasoning loop, model, provider, launcher,
or sandbox. The generic tool contract is documented in
[`docs/agent-arm101-cli.md`](../docs/agent-arm101-cli.md).

## Machine setup

Copy the example setup and replace the local device identifiers:

```bash
cp agent-as-code/setup.example.json agent-as-code/setup.local.json
soarm101 camera list --json
python agent-as-code/capture_observation.py \
  --setup agent-as-code/setup.local.json \
  --check
```

The setup file records the robot serial port and camera device paths used by an experiment.
Prefer stable Linux camera paths under `/dev/v4l/by-id/` when available.

Exactly one enabled camera must have `"primary": true`. Additional enabled cameras are
optional context views.

## Multi-camera capture helper

`capture_observation.py` is only a convenience wrapper around the public camera CLI. It
opens each configured camera sequentially, captures one fresh frame, closes the device, and
writes a JSON manifest:

```bash
python agent-as-code/capture_observation.py \
  --setup agent-as-code/setup.local.json \
  --label test
```

It is not a second camera implementation and does not alter the persisted GUI camera
configuration.

## Agent runs

Agent launch configuration intentionally lives outside this SDK. A run may use Codex,
Hermes, another coding agent, or a custom harness. The SDK-side inputs are simply:

- the task/goal supplied by the experiment;
- the public `soarm101` executable;
- the neutral CLI contract in `docs/agent-arm101-cli.md`; and
- machine-specific connection information needed to address the configured robot/camera.

For runs intended to measure an agent's own task strategy, do not add an SDK-provided
problem-solving policy to those inputs.

The GUI and CLI cannot own the same follower serial port simultaneously. Likewise, a CLI
capture cannot open a USB camera while the GUI owns that camera stream.
