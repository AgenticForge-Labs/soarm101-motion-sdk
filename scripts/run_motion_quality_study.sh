#!/usr/bin/env bash
set -euo pipefail

BRANCH="fix/quantized-slow-tail"

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

echo "SO-ARM101 motion-quality study"
echo "Repository: $ROOT"
echo "Updating branch: $BRANCH"

git fetch origin
git switch "$BRANCH"
git pull --ff-only origin "$BRANCH"

if [[ ! -x .venv/bin/python ]]; then
  echo "Creating local virtual environment..."
  python3 -m venv .venv
fi

# shellcheck disable=SC1091
source .venv/bin/activate

python -m pip install -e ".[gui]" >/dev/null

echo
echo "Running guided experiment from:"
git rev-parse HEAD
echo

exec python examples/motion_quality_walkthrough.py "$@"
