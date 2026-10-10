#!/usr/bin/env bash
# Software-only PR88 checkout/test. Never touches a physical robot.
set -Eeuo pipefail
REPO="https://github.com/AgenticForge-Labs/soarm101-motion-sdk.git"
BRANCH="feature/broker-persistent-sdk-session"
CHECKOUT="${SOARM101_CHECKOUT:-$HOME/AgenticForge/soarm101-motion-sdk}"
LOG_DIR=""
report() {
  status="$?"
  if [[ "$status" -eq 0 ]]; then
    echo "PASS: PR88 simulation checks completed."
  else
    echo "FAIL: PR88 simulation checks exited $status." >&2
  fi
  if [[ -n "$LOG_DIR" ]]; then
    printf 'Logs: %s\n' "$LOG_DIR"
  fi
}
trap report EXIT
command -v git >/dev/null || { echo "Install git." >&2; exit 2; }
command -v uv >/dev/null || { echo "Install uv before testing." >&2; exit 2; }
mkdir -p "$(dirname "$CHECKOUT")"
if [[ -d "$CHECKOUT/.git" ]]; then
  cd "$CHECKOUT"
  [[ -z "$(git status --porcelain)" ]] || {
    echo "Worktree contains local edits; refusing to change branches." >&2
    exit 2
  }
elif [[ -e "$CHECKOUT" ]]; then
  echo "Checkout path exists but is not a Git repository: $CHECKOUT" >&2
  exit 2
else
  git clone "$REPO" "$CHECKOUT"
  cd "$CHECKOUT"
fi
origin="$(git remote get-url origin)"
case "$origin" in
  https://github.com/AgenticForge-Labs/soarm101-motion-sdk|\
https://github.com/AgenticForge-Labs/soarm101-motion-sdk.git|\
git@github.com:AgenticForge-Labs/soarm101-motion-sdk.git)
    ;;
  *) echo "Unexpected origin $origin" >&2; exit 2 ;;
esac
git fetch --prune origin
git show-ref --verify --quiet "refs/remotes/origin/$BRANCH" || exit 2
if git show-ref --verify --quiet "refs/heads/$BRANCH"; then
  git switch "$BRANCH"
else
  git switch --track "origin/$BRANCH"
fi
git pull --ff-only origin "$BRANCH"
[[ "$(git rev-parse HEAD)" == "$(git rev-parse "origin/$BRANCH")" ]] || exit 2
printf 'Checkout: %s\nSHA: %s\n' "$PWD" "$(git rev-parse HEAD)"
uv python install 3.12
uv sync --python 3.12 --extra dev --extra gui --extra mcp --extra simulation
# shellcheck disable=SC1091
source .venv/bin/activate
export QT_QPA_PLATFORM=offscreen
LOG_DIR="$(mktemp -d "${TMPDIR:-/tmp}/soarm101-pr88-XXXXXX")"
ruff check . 2>&1 | tee "$LOG_DIR/ruff.log"
python -m compileall -q src
soarm101-broker --help > "$LOG_DIR/broker-help.txt"
grep -- --sdk-simulation-preview "$LOG_DIR/broker-help.txt"
python -m pytest --no-cov tests/test_broker_sdk.py tests/test_broker.py \
  tests/test_capability_profile.py 2>&1 | tee "$LOG_DIR/broker-tests.log"
python -m pytest 2>&1 | tee "$LOG_DIR/full-tests.log"
soarm101 sdk-capabilities --json > "$LOG_DIR/registry.json"
soarm101-gui --simulation --smoke-test 2>&1 | tee "$LOG_DIR/gui-smoke.log"
echo "No live serial connection, real arm motion, or torque enable was requested."
printf 'For later use: cd %q && source .venv/bin/activate\n' "$PWD"
