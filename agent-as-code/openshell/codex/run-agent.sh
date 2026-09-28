#!/usr/bin/env bash
set -euo pipefail

TASK="${1:-Put the green dragon in the box.}"
MODEL="${2:-}"
REASONING="${AGENT_AS_CODE_CODEX_REASONING:-high}"

mkdir -p "$HOME/.codex" /workspace/runs

if [[ -n "${CODEX_AUTH_ACCESS_TOKEN:-}" && -n "${CODEX_AUTH_REFRESH_TOKEN:-}" && -n "${CODEX_AUTH_ACCOUNT_ID:-}" ]]; then
    python3 - <<'PY'
import base64
import json
import os
import time
from pathlib import Path

def b64u(value):
    return base64.urlsafe_b64encode(json.dumps(value).encode()).rstrip(b"=").decode()

now = int(time.time())
id_token = ".".join([
    b64u({"alg": "none", "typ": "JWT"}),
    b64u({
        "iss": "https://auth.openai.com",
        "aud": "codex",
        "sub": "agenticforge-openshell",
        "iat": now,
        "exp": now + 3600,
    }),
    "placeholder",
])
payload = {
    "auth_mode": "chatgpt",
    "OPENAI_API_KEY": None,
    "tokens": {
        "id_token": id_token,
        "access_token": os.environ["CODEX_AUTH_ACCESS_TOKEN"],
        "refresh_token": os.environ["CODEX_AUTH_REFRESH_TOKEN"],
        "account_id": os.environ["CODEX_AUTH_ACCOUNT_ID"],
    },
    "last_refresh": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
}
path = Path.home() / ".codex" / "auth.json"
path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
path.chmod(0o600)
PY
elif [[ -n "${OPENAI_API_KEY:-}" ]]; then
    printenv OPENAI_API_KEY | codex login --with-api-key >/dev/null
else
    echo "Codex provider did not inject subscription OAuth fields or OPENAI_API_KEY." >&2
    exit 2
fi

PROMPT=$(cat <<EOF
Read /opt/agent-as-code/AGENTS.md and /opt/agent-as-code/CLI.md before acting.

You are controlling a physical SO-ARM101 through the robotctl command only.
Use robotctl observe to acquire fresh images. Use the Codex view_image tool on
the returned local image paths when visual evidence is needed. Do not assume a
physical action worked merely because the command returned successfully.

TASK:
$TASK
EOF
)

ARGS=(exec --skip-git-repo-check --ephemeral)
if codex exec --help 2>/dev/null | grep -q -- "--dangerously-bypass-approvals-and-sandbox"; then
    ARGS+=(--dangerously-bypass-approvals-and-sandbox)
else
    ARGS+=(--sandbox danger-full-access)
fi
if codex exec --help 2>/dev/null | grep -q -- "--ignore-user-config"; then
    ARGS+=(--ignore-user-config)
fi
if codex exec --help 2>/dev/null | grep -q -- "--ignore-rules"; then
    ARGS+=(--ignore-rules)
fi
if [[ -n "$MODEL" ]]; then
    ARGS+=(-c "model=\"$MODEL\"")
fi
ARGS+=(-c "model_reasoning_effort=\"$REASONING\"")

exec codex "${ARGS[@]}" "$PROMPT"
