# Agent as Code

This directory is a small, model-agnostic physical-agent experiment built on the
SO-ARM101 Motion SDK CLI. The experiment intentionally does not depend on a particular
agent harness or vision-language model.

The operating contract is:

```text
natural-language task
        ↓
agent + any VLM available to that agent
        ↓
AGENTS.md + CLI.md + setup.local.json
        ↓
soarm101 CLI
        ↓
guarded Motion SDK + raw USB cameras
        ↓
physical robot
```

The first target task can be as simple as:

> Put the green dragon in the box.

The agent is responsible for deciding when to observe, how to interpret the images,
which guarded CLI commands to issue, and when the task is complete. The SDK remains
responsible for deterministic motion, calibration/provenance checks, safety guards, and
raw camera acquisition.

## Setup

Copy `setup.example.json` to `setup.local.json` and edit the machine-specific
device paths:

```bash
cp agent-as-code/setup.example.json agent-as-code/setup.local.json
soarm101 camera list --json
python agent-as-code/capture_observation.py --setup agent-as-code/setup.local.json --check
```

Prefer stable Linux device paths under `/dev/v4l/by-id/` when available. Exactly one
enabled camera must have `"primary": true`. That is the canonical task view presented
first to the agent/VLM. Any number of additional enabled USB cameras may be listed as
context views. They can be ordinary webcams.

The per-camera settings in this setup are experiment-local. Observation capture passes
them as command-line overrides to `soarm101 camera capture`; it does **not** rewrite
the SDK's persisted GUI camera configuration.

The `agent` and `vlm` fields are provenance labels only for now. They deliberately
do not constrain how the agent or VLM is launched. A run may use a coding agent with
native vision, an agent connected to a separate VLM, a local model, or a hosted model.

## Observations

Capture one fresh frame from every enabled camera:

```bash
python agent-as-code/capture_observation.py \
  --setup agent-as-code/setup.local.json \
  --label before-grasp
```

The helper calls the public `soarm101 camera capture` CLI once per camera, saves the
frames under the configured output directory, writes a JSON manifest, and prints the
manifest path. The primary view appears first in the manifest.

Agents may also call `soarm101 camera capture` directly. The helper exists only to make
a synchronized-enough multi-view observation easy; it is not a second camera
implementation.

## Running an agent

Launch the agent from this directory or otherwise ensure it receives:

- `AGENTS.md` for the physical-operation contract,
- `CLI.md` for the relevant CLI surface,
- `setup.local.json` for the robot and camera devices,
- the natural-language task.

The agent should have access to the `soarm101` executable and to the captured image
files. A multimodal agent must actually receive or open those image files; a pathname by
itself is not visual input.

This first scaffold does not choose an agent API, impose an MCP layer, implement an
evaluator, or claim physical validation. Those can be added after the basic
observe-reason-act loop has been exercised on the real workstation.
