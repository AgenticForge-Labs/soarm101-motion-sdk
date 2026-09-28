#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SECRETS_DIR="${SCRIPT_DIR}/.secrets"
TOKEN_FILE="${SECRETS_DIR}/robot-executor.token"
SETUP_FILE="${SCRIPT_DIR}/setup.local.json"
SERVICE_DIR="${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user"
SERVICE_FILE="${SERVICE_DIR}/agenticforge-robot-executor.service"
EXECUTOR_PORT=8765
CODEX_IMAGE="agenticforge/agent-as-code-codex:local"
HERMES_IMAGE="agenticforge/agent-as-code-hermes:local"

say() { printf "\n==> %s\n" "$*"; }
die() { printf "error: %s\n" "$*" >&2; exit 1; }
need() { command -v "$1" >/dev/null 2>&1 || die "missing required command: $1"; }

[[ "$(uname -s)" == "Linux" ]] || die "the easy installer currently targets Linux/Ubuntu"
need docker
need python3
need curl
need systemctl

if ! command -v uv >/dev/null 2>&1; then
    die "uv is required for this repository. Install it first: https://docs.astral.sh/uv/"
fi

say "Checking Docker"
docker info >/dev/null 2>&1 || die "Docker is installed but not usable by this user"

say "Installing/checking NVIDIA OpenShell"
if ! command -v openshell >/dev/null 2>&1; then
    curl -LsSf https://raw.githubusercontent.com/NVIDIA/OpenShell/main/install.sh | sh
fi
openshell status

say "Installing Motion SDK + camera dependencies"
cd "$REPO_ROOT"
uv sync --extra camera

if [[ ! -f "$SETUP_FILE" ]]; then
    cp "${SCRIPT_DIR}/setup.example.json" "$SETUP_FILE"
    printf "Created %s. Edit the robot and camera device paths before a physical run.\n" "$SETUP_FILE"
fi

say "Creating robot-executor credential"
mkdir -p "$SECRETS_DIR"
chmod 700 "$SECRETS_DIR"
if [[ ! -f "$TOKEN_FILE" ]]; then
    python3 - <<PY
from pathlib import Path
import secrets
path = Path(r"$TOKEN_FILE")
path.write_text(secrets.token_urlsafe(48) + "\n", encoding="utf-8")
path.chmod(0o600)
PY
fi
chmod 600 "$TOKEN_FILE"
ROBOT_EXECUTOR_TOKEN="$(<"$TOKEN_FILE")"
export ROBOT_EXECUTOR_TOKEN

BIND_HOST="${AGENT_AS_CODE_EXECUTOR_BIND:-}"
if [[ -z "$BIND_HOST" ]]; then
    BIND_HOST="$(docker network inspect bridge --format '{{(index .IPAM.Config 0).Gateway}}' 2>/dev/null || true)"
fi
[[ -n "$BIND_HOST" ]] || die "could not detect Docker bridge gateway; set AGENT_AS_CODE_EXECUTOR_BIND"

say "Installing host robot-executor service on $BIND_HOST:$EXECUTOR_PORT"
mkdir -p "$SERVICE_DIR"
cat >"$SERVICE_FILE" <<EOF
[Unit]
Description=AgenticForge constrained robot executor
After=docker.service

[Service]
Type=simple
WorkingDirectory=$REPO_ROOT
ExecStart=$REPO_ROOT/.venv/bin/python $SCRIPT_DIR/host_executor.py --setup $SETUP_FILE --token-file $TOKEN_FILE --bind-host $BIND_HOST --port $EXECUTOR_PORT
Restart=on-failure
RestartSec=2

[Install]
WantedBy=default.target
EOF
systemctl --user daemon-reload
systemctl --user enable --now agenticforge-robot-executor.service
sleep 1
systemctl --user --no-pager --full status agenticforge-robot-executor.service >/dev/null

say "Importing OpenShell provider profiles"
for profile in "${SCRIPT_DIR}"/openshell/providers/*.yaml; do
    openshell profile lint -f "$profile"
    profile_id="$(python3 - "$profile" <<'PY'
import re, sys
for line in open(sys.argv[1], encoding="utf-8"):
    m = re.match(r"^id:\s*([a-z0-9-]+)\s*$", line)
    if m:
        print(m.group(1))
        break
PY
)"
    [[ -n "$profile_id" ]] || die "could not read profile id from $profile"
    if openshell profile describe "$profile_id" >/dev/null 2>&1; then
        printf "Profile already present: %s\n" "$profile_id"
    else
        openshell profile import -f "$profile"
    fi
done

say "Creating/updating OpenShell robot provider"
if openshell provider get agent-as-code-robot >/dev/null 2>&1; then
    openshell provider update agent-as-code-robot --from-existing >/dev/null
else
    openshell provider create \
        --name agent-as-code-robot \
        --type agenticforge-robot-executor \
        --from-existing >/dev/null
fi

say "Configuring Hermes/OpenRouter provider"
if [[ -z "${OPENROUTER_API_KEY:-}" ]] && ! openshell provider get agent-as-code-openrouter >/dev/null 2>&1; then
    read -r -s -p "OpenRouter API key for Hermes (input hidden): " OPENROUTER_API_KEY
    printf "\n"
    export OPENROUTER_API_KEY
fi
if [[ -n "${OPENROUTER_API_KEY:-}" ]]; then
    if openshell provider get agent-as-code-openrouter >/dev/null 2>&1; then
        openshell provider update agent-as-code-openrouter --from-existing >/dev/null
    else
        openshell provider create \
            --name agent-as-code-openrouter \
            --type agenticforge-hermes-openrouter \
            --from-existing >/dev/null
    fi
elif ! openshell provider get agent-as-code-openrouter >/dev/null 2>&1; then
    printf "Hermes provider not configured; rerun with OPENROUTER_API_KEY set.\n"
fi
unset OPENROUTER_API_KEY || true

say "Configuring Codex provider"
CODEX_AUTH_FILE="${HOME}/.codex/auth.json"
if [[ -f "$CODEX_AUTH_FILE" ]]; then
    mapfile -t CODEX_VALUES < <(python3 - "$CODEX_AUTH_FILE" <<'PY'
import json, sys
p = json.load(open(sys.argv[1], encoding="utf-8"))
t = p.get("tokens") or {}
for key in ("access_token", "refresh_token", "account_id"):
    print(t.get(key) or "")
PY
)
    if [[ -n "${CODEX_VALUES[0]:-}" && -n "${CODEX_VALUES[1]:-}" && -n "${CODEX_VALUES[2]:-}" ]]; then
        export CODEX_AUTH_ACCESS_TOKEN="${CODEX_VALUES[0]}"
        export CODEX_AUTH_REFRESH_TOKEN="${CODEX_VALUES[1]}"
        export CODEX_AUTH_ACCOUNT_ID="${CODEX_VALUES[2]}"
        if openshell provider get agent-as-code-codex-subscription >/dev/null 2>&1; then
            openshell provider update agent-as-code-codex-subscription --from-existing >/dev/null
        else
            openshell provider create \
                --name agent-as-code-codex-subscription \
                --type agenticforge-codex-subscription \
                --from-existing >/dev/null
        fi
        unset CODEX_AUTH_ACCESS_TOKEN CODEX_AUTH_REFRESH_TOKEN CODEX_AUTH_ACCOUNT_ID
        printf "Codex subscription provider configured from existing host Codex login.\n"
    fi
fi

if ! openshell provider get agent-as-code-codex-subscription >/dev/null 2>&1; then
    if [[ -n "${OPENAI_API_KEY:-}" ]]; then
        if openshell provider get agent-as-code-codex-api >/dev/null 2>&1; then
            openshell provider update agent-as-code-codex-api --from-existing >/dev/null
        else
            openshell provider create \
                --name agent-as-code-codex-api \
                --type agenticforge-codex-api \
                --from-existing >/dev/null
        fi
        printf "Codex API-key provider configured.\n"
    else
        printf "Codex provider not configured. Run 'codex login' on the host, then rerun this installer.\n"
    fi
fi

say "Building sandbox images"
docker build -t "$CODEX_IMAGE" -f "${SCRIPT_DIR}/openshell/codex/Dockerfile" "$SCRIPT_DIR"
docker build -t "$HERMES_IMAGE" -f "${SCRIPT_DIR}/openshell/hermes/Dockerfile" "$SCRIPT_DIR"

say "Validating local setup structure"
"$REPO_ROOT/.venv/bin/python" "${SCRIPT_DIR}/capture_observation.py" --setup "$SETUP_FILE" --check

cat <<EOF

Installed.

Before physical motion:
  1. Edit: $SETUP_FILE
  2. Review: $SCRIPT_DIR/AGENTS.md
  3. Run:    $SCRIPT_DIR/doctor.sh
  4. Codex:  $SCRIPT_DIR/run-codex.sh
  5. Hermes: $SCRIPT_DIR/run-hermes.sh

Hermes defaults to deepseek/deepseek-v4.1-flash through OpenRouter.
No sandbox receives direct access to the serial port or USB cameras.
EOF
