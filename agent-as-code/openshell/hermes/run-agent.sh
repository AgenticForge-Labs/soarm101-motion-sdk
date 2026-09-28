#!/usr/bin/env bash
set -euo pipefail

TASK="${1:-Put the green dragon in the box.}"
MODEL="${2:-deepseek/deepseek-v4.1-flash}"

mkdir -p /workspace/runs /opt/data
PROMPT_FILE="/tmp/agent-as-code-task.md"
cat >"$PROMPT_FILE" <<EOF
Read /opt/agent-as-code/AGENTS.md and /opt/agent-as-code/CLI.md before acting.

You are controlling a physical SO-ARM101 through the robotctl command only.
Use robotctl observe to acquire fresh camera images. Inspect the returned local
image paths with your vision capability before deciding on visually grounded
actions. Re-observe after meaningful physical changes.

TASK:
$TASK
EOF

exec hermes chat \
    --provider openrouter \
    --model "$MODEL" \
    --toolsets "file,terminal,vision" \
    --query-file "$PROMPT_FILE"
