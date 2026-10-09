#!/usr/bin/env bash
# Pull, install, and validate PR #87. Software/simulation ONLY: never activate a real arm.
#
# Bootstrap with a single shell line:
# curl -fsSL https://raw.githubusercontent.com/AgenticForge-Labs/soarm101-motion-sdk/feature/shared-sdk-capability-registry/scripts/test_registry_pr87.sh -o /tmp/test_registry_pr87.sh && bash /tmp/test_registry_pr87.sh
#
# Or run from an existing checkout: bash scripts/test_registry_pr87.sh
set -Eeuo pipefail

REPO_URL="https://github.com/AgenticForge-Labs/soarm101-motion-sdk.git"
BRANCH="feature/shared-sdk-capability-registry"
# Last CI-verified base of the registry implementation, not necessarily the latest head.
GREEN_BASE_SHA="c28921b9c3d9318bd1c203c991ea5f3d60f6681c"
CHECKOUT="${SOARM101_CHECKOUT:-${1:-$HOME/AgenticForge/soarm101-motion-sdk}}"
LOG_DIR=""

on_exit() {
    local status="$?"
    if [[ "$status" -eq 0 ]]; then
        echo "PASS: all software-only checks completed."
    else
        printf 'FAIL: test exited with status %s\n' "$status" >&2
    fi
    if [[ -n "$LOG_DIR" ]]; then
        printf 'Test logs: %s\n' "$LOG_DIR"
    fi
}
trap on_exit EXIT

for tool in git curl; do
    command -v "$tool" >/dev/null 2>&1 || {
        printf 'Missing required command: %s\n' "$tool" >&2
        exit 2
    }
done

mkdir -p "$(dirname "$CHECKOUT")"
if [[ -d "$CHECKOUT/.git" ]]; then
    cd "$CHECKOUT"
    if [[ -n "$(git status --porcelain)" ]]; then
        echo 'Checkout contains local changes. Commit or stash them before pulling.' >&2
        exit 2
    fi
elif [[ -e "$CHECKOUT" ]]; then
    printf 'Path exists but is not a Git checkout: %s\n' "$CHECKOUT" >&2
    exit 2
else
    git clone "$REPO_URL" "$CHECKOUT"
    cd "$CHECKOUT"
fi

origin="$(git remote get-url origin)"
case "$origin" in
    https://github.com/AgenticForge-Labs/soarm101-motion-sdk|\
https://github.com/AgenticForge-Labs/soarm101-motion-sdk.git|\
git@github.com:AgenticForge-Labs/soarm101-motion-sdk.git|\
ssh://git@github.com/AgenticForge-Labs/soarm101-motion-sdk.git) ;;
    *) printf 'Unexpected origin: %s\n' "$origin" >&2; exit 2 ;;
esac
git fetch --prune origin
if ! git show-ref --verify --quiet "refs/remotes/origin/$BRANCH"; then
    echo "Missing remote branch origin/$BRANCH" >&2
    exit 2
fi
if git show-ref --verify --quiet "refs/heads/$BRANCH"; then
    git switch "$BRANCH"
else
    git switch --track "origin/$BRANCH"
fi
git pull --ff-only origin "$BRANCH"

current_sha="$(git rev-parse HEAD)"
if [[ "$current_sha" != "$(git rev-parse "origin/$BRANCH")" ]]; then
    echo 'Local branch is ahead/diverged from the remote. Refusing ambiguous test.' >&2
    exit 3
fi
if ! git merge-base --is-ancestor "$GREEN_BASE_SHA" "$current_sha"; then
    echo 'Checked-out branch does not contain the previously CI-verified baseline.' >&2
    exit 3
fi
printf '\nCheckout: %s\nRevision: %s\n' "$PWD" "$current_sha"
echo 'The pinned baseline passed CI; the current branch revision is tested below.'

if command -v uv >/dev/null 2>&1; then
    uv python install 3.12
    uv sync --python 3.12 --extra dev --extra gui --extra simulation --extra mcp
else
    python_bin=""
    for candidate in python3.12 python3.11 python3.10 python3; do
        if command -v "$candidate" >/dev/null 2>&1 &&
           "$candidate" -c 'import sys; sys.exit(0 if (3,10) <= sys.version_info[:2] < (3,14) else 1)'; then
            python_bin="$candidate"
            break
        fi
    done
    if [[ -z "$python_bin" ]]; then
        echo 'Install uv or Python 3.10-3.13 before running.' >&2
        exit 2
    fi
    "$python_bin" -m venv .venv
    .venv/bin/python -m pip install -e '.[dev,gui,simulation,mcp]'
fi

# Activation persists only inside this script, not the user's interactive shell.
# shellcheck disable=SC1091
source .venv/bin/activate
printf '\nActivated: %s\n' "$(command -v python)"
python --version

LOG_DIR="$(mktemp -d "${TMPDIR:-/tmp}/soarm101-registry-XXXXXX")"
printf 'Test logs: %s\n' "$LOG_DIR"
export QT_QPA_PLATFORM=offscreen

ruff check . 2>&1 | tee "$LOG_DIR/ruff.log"
python -m compileall -q src
soarm101 info 2>&1 | tee "$LOG_DIR/info.log"
soarm101 sdk-capabilities --json | tee "$LOG_DIR/sdk-capabilities.json"
soarm101 read --simulation --json | tee "$LOG_DIR/simulation-read.json"
python -m pytest --no-cov tests/test_sdk_capabilities.py tests/test_cli.py \
    2>&1 | tee "$LOG_DIR/focused-tests.log"
python -m pytest 2>&1 | tee "$LOG_DIR/full-tests.log"
soarm101-gui --simulation --smoke-test 2>&1 | tee "$LOG_DIR/gui-smoke.log"
soarm101 sim-demo 2>&1 | tee "$LOG_DIR/sim-demo.log"

echo 'No physical serial connection, real-arm motion, or torque enable was requested.'
echo 'To activate for later interactive commands:'
printf '  cd %q && source .venv/bin/activate\n' "$PWD"
