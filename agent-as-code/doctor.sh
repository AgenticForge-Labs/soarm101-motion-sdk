#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
TOKEN_FILE="${SCRIPT_DIR}/.secrets/robot-executor.token"
SETUP_FILE="${SCRIPT_DIR}/setup.local.json"
PORT=8765

printf "Agent-as-code doctor\n\n"
command -v openshell >/dev/null
command -v docker >/dev/null
openshell status
docker info >/dev/null

"$REPO_ROOT/.venv/bin/python" "${SCRIPT_DIR}/capture_observation.py" --setup "$SETUP_FILE" --check

BIND_HOST="${AGENT_AS_CODE_EXECUTOR_BIND:-$(docker network inspect bridge --format '{{(index .IPAM.Config 0).Gateway}}')}"
TOKEN="$(<"$TOKEN_FILE")"
ROBOT_EXECUTOR_TOKEN="$TOKEN" ROBOT_EXECUTOR_URL="http://$BIND_HOST:$PORT" \
    "$REPO_ROOT/.venv/bin/python" "${SCRIPT_DIR}/robotctl.py" health

systemctl --user is-active --quiet agenticforge-robot-executor.service
openshell provider get agent-as-code-robot >/dev/null

if openshell provider get agent-as-code-codex-subscription >/dev/null 2>&1; then
    printf "Codex provider: subscription OAuth bridge\n"
elif openshell provider get agent-as-code-codex-api >/dev/null 2>&1; then
    printf "Codex provider: OpenAI API key\n"
else
    printf "Codex provider: NOT CONFIGURED\n"
fi

if openshell provider get agent-as-code-openrouter >/dev/null 2>&1; then
    printf "Hermes provider: OpenRouter configured\n"
else
    printf "Hermes provider: NOT CONFIGURED\n"
fi

printf "\nHost/sandbox prerequisites look ready. This does not move the robot or open cameras.\n"
