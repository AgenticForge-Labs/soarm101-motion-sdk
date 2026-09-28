# NVIDIA OpenShell boundary

`agent-as-code` uses NVIDIA OpenShell as the common agent security boundary.

## Why the hardware stays outside the sandbox

Codex, Hermes, and future agents should be compared with the same physical capabilities.
Passing serial/camera devices directly into each agent image would couple the benchmark to
agent-specific permissions and would let an agent invent an alternate hardware path.

Instead:

```text
OpenShell sandbox
  agent
    ↓
  robotctl
    ↓
  endpoint-bound credential + REST policy
    ↓
host robot executor
    ↓
soarm101 CLI
    ↓
Motion SDK
    ↓
USB camera / SO-ARM101
```

The host executor is intentionally not a second robotics implementation. It validates the
outer benchmark envelope and then constructs existing `soarm101` commands.

## Shared OpenShell policy

Both agent images use `openshell/policy.yaml`. It provides a writable sandbox workspace
and the runtime paths needed by the image. It does not itself grant external network
access.

Attached provider profiles add only the endpoints appropriate to that run:

- `agenticforge-robot-executor`: `host.openshell.internal:8765`, limited to the six
  documented REST routes and Python client binaries.
- `agenticforge-hermes-openrouter`: OpenRouter, limited to the Hermes runtime.
- `agenticforge-codex-subscription`: OpenAI/ChatGPT endpoints needed by a placeholder-
  backed Codex subscription login.
- `agenticforge-codex-api`: OpenAI API endpoint for API-key Codex.

The random robot-executor bearer credential is generated locally, stored under
`agent-as-code/.secrets/`, and imported into OpenShell's credential store. The sandbox
receives the OpenShell placeholder rather than the host token value.

## Codex subscription note

Current OpenShell does not yet provide a fully gateway-owned Sign in with ChatGPT
lifecycle for Codex subscription OAuth. This project therefore follows NVIDIA's existing
demonstrated bridge pattern: extract the already-authenticated host Codex access token,
refresh token, and account ID into an endpoint-bound provider, then materialize a
placeholder-backed `~/.codex/auth.json` inside the sandbox.

This is appropriate for short benchmark runs, but it is not presented as a permanent
OAuth lifecycle implementation. If the host login changes or refresh behavior fails,
sign in on the host and rerun `install.sh`.

Using `OPENAI_API_KEY` avoids that subscription bridge and uses the simpler API-key
provider.

## Agent-specific inner sandboxes

Codex's internal sandbox/approval layer is disabled for the benchmark run when the
installed CLI supports that option. This does **not** remove the outer OpenShell policy.
It prevents Codex from receiving a stricter effective tool permission set than Hermes.

Hermes uses `TERMINAL_ENV=local`, meaning its terminal executes in its OpenShell
container rather than creating a second execution environment.

## Host exposure

The installer binds the host executor to Docker's bridge gateway address instead of
`0.0.0.0` by default. OpenShell sandboxes reach that host through
`host.openshell.internal`. Requests still require the random bearer credential and must
match the provider's endpoint/method policy.

If Docker networking is customized, set `AGENT_AS_CODE_EXECUTOR_BIND` to the host
address reachable through the OpenShell Docker network and rerun `install.sh`.
