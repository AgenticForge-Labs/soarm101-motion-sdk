#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SETUP="${SCRIPT_DIR}/setup.local.json"
TASK_FILE="${1:-}"
MODEL="${2:-}"

[[ -f "$SETUP" ]] || { echo "missing $SETUP; run ./agent-as-code/install.sh" >&2; exit 2; }
openshell provider get agent-as-code-openrouter >/dev/null 2>&1 || {
    echo "Hermes/OpenRouter provider is not configured; run ./agent-as-code/install.sh" >&2
    exit 2
}

readarray -t CFG < <(python3 - "$SETUP" "$TASK_FILE" "$MODEL" <<'PY'
import json, pathlib, sys
setup_path = pathlib.Path(sys.argv[1])
setup = json.loads(setup_path.read_text())
task_arg, model_arg = sys.argv[2], sys.argv[3]
task_ref = task_arg or setup.get("experiment", {}).get("task_file", "TASK.example.md")
task_path = pathlib.Path(task_ref)
if not task_path.is_absolute():
    task_path = setup_path.parent / task_path
print(task_path)
print(model_arg or setup.get("agents", {}).get("hermes", {}).get("model", "deepseek/deepseek-v4.1-flash"))
PY
)
TASK_PATH="${CFG[0]}"
MODEL="${CFG[1]}"
[[ -f "$TASK_PATH" ]] || { echo "task file not found: $TASK_PATH" >&2; exit 2; }
TASK="$(cat "$TASK_PATH")"

RUN_ID="hermes-$(date +%Y%m%d-%H%M%S)"
RUN_DIR="${SCRIPT_DIR}/runs/$RUN_ID"
mkdir -p "$RUN_DIR"
python3 - "$RUN_DIR/run.json" "$MODEL" "$TASK_PATH" <<'PY'
import json, pathlib, sys, time
pathlib.Path(sys.argv[1]).write_text(json.dumps({
    "agent": "hermes",
    "provider": "openrouter",
    "model": sys.argv[2],
    "task_file": sys.argv[3],
    "started_at": time.time(),
}, indent=2) + "\n")
PY

KEEP=()
[[ "${AGENT_AS_CODE_KEEP_SANDBOX:-0}" == "1" ]] || KEEP+=(--no-keep)

openshell sandbox create \
    --name "$RUN_ID" \
    --from agenticforge/agent-as-code-hermes:local \
    --policy "${SCRIPT_DIR}/openshell/policy.yaml" \
    --provider agent-as-code-robot \
    --provider agent-as-code-openrouter \
    --no-auto-providers \
    --no-tty \
    "${KEEP[@]}" \
    -- /usr/local/bin/run-agent-as-code-hermes "$TASK" "$MODEL" \
    2>&1 | tee "$RUN_DIR/agent.log"
