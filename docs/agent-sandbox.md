# Self-contained agent sandbox

The Motion SDK ships a canonical OpenShell environment for operating an SO-ARM101 with a
reasoning agent without exposing the unrestricted SDK, serial bus, or camera devices to that
agent.

Hermes and Codex are first-class packaged adapters:

- **Hermes** — OpenRouter-backed, with Hermes `vision_analyze`.
- **Codex CLI** — OpenAI Platform API-key mode, reuse of the existing installed Codex
  ChatGPT login, or an optional separate SDK-owned ChatGPT login; Codex uses `view_image`.

OpenShell itself can host other CLI agents. The SDK therefore also accepts an explicit,
operator-authored JSON adapter manifest for a harness/image already usable by OpenShell.
Packaged and manifest-defined adapters use exactly the same robot boundary:

```text
human
  -> soarm101 agent arm
  -> soarm101 agent sandbox run --agent AGENT [--adapter-manifest FILE]
       -> OpenShell
       -> selected agent harness
       -> SKILL.md + robotctl.py
       -> host-side soarm101 broker
       -> bounded soarm101 agent capabilities
       -> Motion SDK
       -> robot + named cameras
```

OpenShell is external infrastructure, not vendored into the Python package. The SDK owns the
robot-specific sandbox policy, adapter contract, broker lifecycle, standalone `robotctl`,
and the packaged Hermes/Codex recipes. An external manifest selects an existing OpenShell
image/provider and direct harness argv; it does not define robot behavior.

## Agent adapter boundary

Agent-specific behavior is isolated in `soarm101_motion.agent_adapters`.

An adapter owns only:

- sandbox image recipe;
- OpenShell provider/profile;
- default model behavior;
- agent-specific skill;
- immutable runtime paths;
- runtime user/group;
- mutable home/config directories;
- command construction; and
- environment variables.

The robot contract is not duplicated per agent. Hermes, Codex, and manifest-defined external
adapters all use the same task upload, `robotctl.py`, broker routes, authority checks,
evidence, and cleanup.

## Security boundary

The sandbox does not receive:

- the Motion SDK source tree;
- the unrestricted `soarm101` CLI;
- serial devices;
- camera devices;
- calibration files;
- Docker socket;
- SSH keys; or
- unrelated host files.

The sandbox receives only the task, selected agent skill, standalone `robotctl.py`, and the
minimal mutable configuration needed by that agent.

The broker exposes only:

```text
GET  /v1/health
GET  /v1/capabilities
GET  /v1/state

POST /v1/capture
POST /v1/go-pose
POST /v1/joint
POST /v1/jog
POST /v1/gripper
POST /v1/sleep
POST /v1/stop
```

There is no remote arm, disarm, relax, calibration, configuration, arbitrary command,
arbitrary shell, raw joint-vector, or servo-register endpoint.

A human creates motion authority separately. The sandbox cannot arm the robot.

OpenShell sandbox creation uses `--no-auto-providers`: only the provider selected for the
chosen agent is attached. Unrelated GitHub/cloud/provider credentials on the workstation are
not inherited.

## Credentials

Use dedicated revocable keys for the sandbox agents.

Hermes setup discovers:

```bash
export OPENROUTER_API_KEY=...
```

Codex has two primary authentication choices plus one optional isolated-login variant.

**OpenAI Platform API key**:

```bash
export OPENAI_API_KEY=...
```

This is the stronger credential-isolation option. Setup stores the key in an OpenShell
provider and normal runs do not place the API key in the sandbox task bundle or Codex home.

**Reuse the already-installed Codex CLI session**:

```bash
soarm101 agent sandbox setup --agent codex --auth installed
```

This reuses the normal Codex CLI ChatGPT login at `${CODEX_HOME:-~/.codex}/auth.json`.
The SDK does **not** mount or modify the normal Codex home. Each run copies only
`auth.json` into temporary staging and uploads that one-run copy to the disposable sandbox.
If that login is missing, run `codex login` in your normal terminal first.

This is the path for using the ChatGPT/Codex subscription you already use with your installed
Codex CLI.

**Optional separate SDK-owned ChatGPT login**:

```bash
soarm101 agent sandbox setup --agent codex --auth chatgpt
```

This mode keeps a separate Codex device-login state for the Motion SDK instead of using the
normal host Codex session. It is stored at:

```text
~/.local/state/soarm101/codex-chatgpt/auth.json
```

or under `$XDG_STATE_HOME/soarm101/codex-chatgpt/auth.json` when configured.

For either installed-login or dedicated-ChatGPT runs, the selected auth file is copied to a temporary staging file and
then uploaded as `/sandbox/.codex/auth.json`. It is marked as a sensitive input, is not
content-hashed into run metadata, and disappears when the disposable OpenShell sandbox is
deleted. Because Codex itself must read and refresh its OAuth state, code running as the same
sandbox user can technically read this one-run copy. For highly untrusted benchmark prompts,
prefer API-key mode with OpenShell credential mediation. ChatGPT mode is intended for
personal/supervised use where using the ChatGPT subscription is desirable.

`--reauth` is not used with `--auth installed`; the normal host Codex CLI owns that
login lifecycle. Force a fresh dedicated SDK-owned ChatGPT login with:

```bash
soarm101 agent sandbox setup --agent codex --auth chatgpt --reauth
```

## One-time setup

Check one agent:

```bash
soarm101 agent sandbox doctor --agent hermes
soarm101 agent sandbox doctor --agent codex
```

Set up Hermes:

```bash
export OPENROUTER_API_KEY=...
soarm101 agent sandbox setup --agent hermes
```

Set up Codex with a dedicated OpenAI Platform API key:

```bash
export OPENAI_API_KEY=...
soarm101 agent sandbox setup --agent codex --auth api-key
```

Or reuse your already-installed/logged-in Codex CLI session:

```bash
soarm101 agent sandbox setup --agent codex --auth installed
```

Optional: create a separate SDK-owned ChatGPT device login:

```bash
soarm101 agent sandbox setup --agent codex --auth chatgpt
```

Setup is idempotent for each agent. It always verifies the OpenShell gateway and builds the
selected canonical image. Provider-backed modes then lint/import/update the packaged OpenShell provider and discover
the matching host credential. Installed mode validates and reuses the host file-backed Codex
ChatGPT login without modifying it. Dedicated `chatgpt` mode uses SDK-owned login state and
creates a device login only when needed.

Canonical defaults are:

```text
Hermes image:    agenticforge/soarm101-hermes-openshell:local
Hermes provider: soarm101-hermes-openrouter

Codex image:          agenticforge/soarm101-codex-openshell:local
Codex API provider:   soarm101-codex-openai-api-key
Codex installed login: host `${CODEX_HOME:-~/.codex}/auth.json`, copied per run
Codex dedicated login: SDK-owned ChatGPT device-login state
```

Re-run:

```bash
soarm101 agent sandbox doctor --agent hermes --json
soarm101 agent sandbox doctor --agent codex --auth api-key --json
soarm101 agent sandbox doctor --agent codex --auth installed --json
soarm101 agent sandbox doctor --agent codex --auth chatgpt --json
```

## Validate without motion

Before powered autonomous work, validate either harness with a hard read-only run:

```bash
soarm101 agent sandbox run --agent hermes --read-only
soarm101 agent sandbox run --agent codex --auth api-key --read-only
soarm101 agent sandbox run --agent codex --auth installed --read-only
soarm101 agent sandbox run --agent codex --auth chatgpt --read-only
```

When `--task` is omitted in read-only mode, the SDK uploads the same harness-neutral
validation task for every agent. It requires the agent to inspect capabilities and state,
capture every configured camera, and use the image-inspection method named by that agent's
skill when available.

Read-only mode does not require `soarm101 agent arm`. An absent authority lease blocks
motion only; it must not stop capabilities/state/camera observation. The generated OpenShell
policy omits every motion route: saved-pose motion, joint motion, Cartesian jog, gripper,
Sleep, and STOP/HOLD are unreachable from the sandbox. Only health/capabilities/state and
fresh camera capture remain available.

The SDK validates this gate deterministically after the harness exits. Only broker events
created after agent handoff count: successful capabilities and state reads are required,
every configured camera must have a successful capture, no motion action may reach the
broker, and every retained observation must match a trusted broker capture SHA-256. The
result is written to `read-only-validation.json`; a zero harness exit without this evidence
fails the run.

The agent-specific image tool differs:

- Hermes skill: `vision_analyze`.
- Codex skill: `view_image`.

## Run a full-control task

Authorize motion from a human terminal:

```bash
soarm101 agent arm --minutes 30
```

Then choose the harness:

```bash
soarm101 agent sandbox run \
  --agent hermes \
  --task TASK.md \
  --output runs/hermes-001
```

or:

```bash
soarm101 agent sandbox run \
  --agent codex \
  --auth api-key \
  --task TASK.md \
  --output runs/codex-api-001

soarm101 agent sandbox run \
  --agent codex \
  --auth installed \
  --task TASK.md \
  --output runs/codex-installed-001

# Optional separate SDK-owned ChatGPT login:
soarm101 agent sandbox run \
  --agent codex \
  --auth chatgpt \
  --task TASK.md \
  --output runs/codex-chatgpt-001
```

### Model selection

Hermes defaults to:

```text
deepseek/deepseek-v4.1-flash
```

Override it with `--model`.

Codex leaves model selection to the Codex CLI default when `--model` is omitted. An
explicit Codex model can also be supplied with `--model`.

This makes a simple same-task comparison possible without adding a benchmark framework:

```bash
soarm101 agent sandbox run --agent hermes --task TASK.md --output runs/h1
soarm101 agent sandbox run --agent codex --auth api-key --task TASK.md --output runs/c-api
soarm101 agent sandbox run --agent codex --auth installed --task TASK.md --output runs/c-installed
```

Higher-level replication, randomization, scoring, statistics, Harbor integration, or
OpenTelemetry aggregation remain outside the device SDK.

## Harness execution details

Hermes runs a finite one-shot `hermes chat` invocation with bundled/project skills disabled
and only the SDK-supplied skill/task in the sandbox workspace.

Codex runs non-interactively with `codex exec --json --ephemeral`. Its user/project config
and rules are ignored. Codex's own command-approval/internal-sandbox layer is disabled because
the entire Codex process already runs inside the stricter OpenShell sandbox.

API-key mode uses an inspected OpenShell REST provider for `api.openai.com` and forces the
Responses API onto HTTPS/SSE (`supports_websockets=false`). Installed and dedicated-ChatGPT
modes deliberately leave model routing to Codex's native first-party provider while supplying
the one-run ChatGPT `auth.json` copy. OpenShell still constrains the Codex executable to
`chatgpt.com:443` and `auth.openai.com:443` for inference and token refresh; automatic
provider attachment remains disabled.

Neither mode broadens robot authority: robot/camera access still has to cross the same
authenticated broker allowlist and deterministic Motion SDK safety checks.

## Agent capability

Inside the sandbox the selected harness can use its normal reasoning/shell/filesystem capabilities.
Physical observation and action remain limited to:

```bash
python3 robotctl.py capabilities
python3 robotctl.py state
python3 robotctl.py capture overhead
python3 robotctl.py capture wrist
python3 robotctl.py go-pose agent_start_overhead
python3 robotctl.py joint shoulder_pan --delta-deg 5
python3 robotctl.py jog --frame world --x-mm 0 --y-mm 5 --z-mm 0
python3 robotctl.py jog --frame tool --x-mm 0 --y-mm 0 --z-mm 2
python3 robotctl.py gripper open
python3 robotctl.py gripper close
python3 robotctl.py sleep
python3 robotctl.py stop
```

World-direction semantics come from measured workspace calibration reported by
`capabilities`. Tool-frame jog follows current TCP axes.

## Evidence and output

Each run records which harness/model/image/provider was requested in:

```text
run-metadata.json
```

The host output directory retains, when produced:

```text
openshell-policy.yaml
openshell-effective-policy.yaml
openshell-create.json
openshell-logs.txt
broker-events.jsonl
broker-stdout.txt
broker-stderr.txt
hermes-stdout.jsonl / codex-stdout.jsonl
hermes-stderr.txt   / codex-stderr.txt
observations/
task-result.json
download-errors.txt
sandbox-cleanup-error.txt
```

Broker capture evidence contains trusted image SHA-256 values. `robotctl.py` verifies those
hashes before writing image bytes into the sandbox. The requested/effective OpenShell policy
and redacted OpenShell logs remain inspectable after the run. The per-run broker token is
redacted from retained logs.

SDK-selected uploads use `--no-git-ignore`, so repository ignore rules cannot silently drop
TASK/SKILL/client/config inputs.

## Adding another agent

Use a packaged Python adapter when the SDK needs harness-specific setup, credential handling,
or optimized image/tool instructions. Hermes and Codex use this path.

For a CLI agent that OpenShell can already run, prefer an external JSON adapter manifest
instead of changing Motion SDK robot code. See
`agent-as-code/openshell-adapter.example.json`. The schema is version 1 and defines only
harness/runtime concerns:

- `name` and the OpenShell `image`;
- optional existing OpenShell `provider`;
- direct `command` argv containing a standalone `{prompt}` token;
- direct `version_command` argv;
- optional `{model}`, `{max_turns}`, `{task_path}`, and `{skill_path}` argv tokens;
- runtime user/group, immutable paths, mutable sandbox directories, and minimal environment;
- the Python/runtime executable path allowed to reach the robot broker; and
- optionally a harness-specific skill file next to the manifest.

Example:

```bash
soarm101 agent sandbox doctor \
  --agent my-agent \
  --adapter-manifest agent-as-code/openshell-adapter.example.json

soarm101 agent sandbox run \
  --agent my-agent \
  --adapter-manifest agent-as-code/openshell-adapter.example.json \
  --read-only
```

Manifest commands are argv arrays, not shell command strings. Shell executables are rejected,
broker URL/token environment variables are runtime-owned, and the manifest cannot add robot
routes or create motion authority. If an external adapter names a provider, that provider must
already exist in OpenShell; the SDK does not invent provider credentials for arbitrary
harnesses.

Do not add agent-specific robot endpoints or bypass the broker merely because another harness
has different tool conventions.

## Validation status

Software tests validate adapter selection, policy construction, packaged assets, CLI wiring,
authority preflight, upload boundaries, broker lifecycle, evidence, and cleanup without
hardware.

Real OpenShell and physical robot validation remain separate gates. Validate in this order for
each packaged or manifest-defined adapter you intend to operate:

1. `sandbox doctor --agent AGENT`;
2. `sandbox setup --agent AGENT`;
3. `sandbox run --agent AGENT --read-only`;
4. inspect the requested/effective policy and fresh camera evidence;
5. after explicit human `agent arm`, run a supervised known `agent_*` pose;
6. small single-joint and world/tool-frame corrections;
7. gripper motion; then
8. a complete manipulation task.

Software STOP/HOLD is not an emergency stop. Keep physical power accessible during every
hardware validation.
