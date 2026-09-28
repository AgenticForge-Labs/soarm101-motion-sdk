# Agent as Code

This directory is a small physical-agent benchmark built on the SO-ARM101 Motion SDK.
The benchmark is intentionally model-agnostic, while NVIDIA OpenShell provides one common
permission boundary for every agent.

The current reference agents are:

- **Codex**, using either an existing host Codex/ChatGPT subscription login or an
  `OPENAI_API_KEY`.
- **Hermes Agent**, using OpenRouter and the vision-capable
  `deepseek/deepseek-v4.1-flash` model by default.

The architecture is deliberately split:

```text
natural-language task
        ↓
Codex OR Hermes + VLM
        ↓
NVIDIA OpenShell sandbox
        ↓
robotctl
        ↓ authenticated, narrowly scoped HTTP
host-side robot executor
        ↓
existing soarm101 CLI
        ↓
guarded Motion SDK + USB cameras
        ↓
physical robot
```

The sandbox never receives the follower serial device or USB camera devices. OpenShell
controls filesystem, process, network, and model-provider access; the host executor owns
the consequential physical interface and accepts only the small set of actions documented
in `CLI.md`.

## One-command install

On the Ubuntu workstation, from the repository root:

```bash
./agent-as-code/install.sh
```

The installer:

1. checks Docker, Python, `uv`, and systemd user services;
2. installs NVIDIA OpenShell with NVIDIA's official installer when needed;
3. installs this repository with the camera extra;
4. creates `agent-as-code/setup.local.json` when absent;
5. generates a local random robot-executor credential;
6. installs and starts the user-level host executor service;
7. lints/imports the narrow OpenShell provider profiles;
8. creates the robot provider and the Hermes/OpenRouter provider;
9. detects an existing host Codex subscription login at `~/.codex/auth.json` and stores
   its sensitive OAuth fields in OpenShell, or falls back to `OPENAI_API_KEY`;
10. builds local Codex and Hermes sandbox images.

The installer prompts once for an OpenRouter API key when Hermes has not yet been
configured. It does not write that key into this repository.

OpenShell's current local runtime requires a usable Docker installation. The installer
uses NVIDIA's normal local gateway and Docker-backed sandboxes.

## Configure the physical setup

Edit:

```text
agent-as-code/setup.local.json
```

Set the SO-ARM101 serial port and camera device paths. Prefer stable Linux camera paths
under `/dev/v4l/by-id/` when available.

Exactly one enabled camera must have `"primary": true`. Any number of additional enabled
USB webcams may be listed. The primary view is returned first to the agent, while the
others are additional evidence.

The `executor` section is also the benchmark's outer physical-action envelope. Port 8765 is\ncurrently fixed because it is part of the endpoint-scoped OpenShell provider profile. The
default is intentionally conservative: one jog request may translate at most 10 mm or
rotate at most 5 degrees, at no more than 10 mm/s and 40 mm/s². The Motion SDK applies
its own calibration, path, joint, fault, following-error, effort, and provenance guards
below these limits.

## Check without moving the robot

After editing the setup:

```bash
./agent-as-code/doctor.sh
```

The doctor checks OpenShell, Docker, the setup file, executor health, and provider
presence. It does **not** open cameras or command robot motion.

For a camera-only host check:

```bash
python agent-as-code/capture_observation.py \
  --setup agent-as-code/setup.local.json \
  --label camera-check
```

Do not run the desktop GUI camera stream at the same time if it owns one of those devices.

## Run Codex

The default task comes from `TASK.example.md`:

```bash
./agent-as-code/run-codex.sh
```

Pass a different task file and optionally a model:

```bash
./agent-as-code/run-codex.sh agent-as-code/my-task.md MODEL
```

Leaving the Codex model blank in `setup.local.json` lets the installed Codex CLI choose
its normal default.

For subscription-backed Codex, first sign in normally on the host:

```bash
codex login
./agent-as-code/install.sh
```

The installer reads only the access token, refresh token, and account ID needed by the
OpenShell provider. The sandbox receives OpenShell placeholders rather than those host
secret values. The sandbox wrapper materializes the placeholder-backed `auth.json`
shape used by NVIDIA's demonstrated OpenShell/Codex OAuth pattern.

If the host Codex login is refreshed or replaced and sandbox Codex authentication later
fails, rerun `install.sh` to update the OpenShell provider.

## Run Hermes + DeepSeek VLM

```bash
./agent-as-code/run-hermes.sh
```

The default is:

```text
provider: OpenRouter
model:    deepseek/deepseek-v4.1-flash
toolsets: file, terminal, vision
```

Override the task and model the same way:

```bash
./agent-as-code/run-hermes.sh agent-as-code/my-task.md PROVIDER_MODEL_ID
```

Hermes runs with its terminal backend local to the OpenShell sandbox. It can write its
sandbox workspace and inspect observations, but it cannot open the physical devices.

## What the agent can do

Inside either sandbox, the agent receives the same robot interface:

```bash
robotctl health
robotctl observe --label initial
robotctl state
robotctl diagnose
robotctl jog --x-mm 2
robotctl gripper open
robotctl gripper close
```

`robotctl observe` asks the host executor to acquire fresh images from all configured
cameras, transfers those images into the sandbox, and prints the local paths. Codex can
attach those paths with its image-viewing capability. Hermes can inspect them through its
vision toolset.

The agent cannot call `soarm101` directly from the sandbox. That is intentional. The
host executor translates the approved requests into the existing public CLI so there is
still one authoritative motion implementation.

## Benchmark runs

Each launcher creates a timestamped host directory under `agent-as-code/runs/` with
basic run metadata and the agent log. Host-side camera acquisition also retains its
observation manifests and images there.

By default the OpenShell sandbox is deleted when the agent exits. To keep it for
inspection:

```bash
AGENT_AS_CODE_KEEP_SANDBOX=1 ./agent-as-code/run-codex.sh
```

or the equivalent Hermes command.

This stage standardizes permissions and execution. Independent task scoring, richer run
metrics, random scene generation, and additional agent adapters can be added without
changing the physical safety boundary.
