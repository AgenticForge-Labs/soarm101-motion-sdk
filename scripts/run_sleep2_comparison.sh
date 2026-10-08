#!/usr/bin/env bash
set -euo pipefail

BRANCH="fix/quantized-slow-tail"
SPEED_DEG_S="${1:-24}"
ACCEL_DEG_S2="${2:-150}"

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

git fetch origin
git switch "$BRANCH"
git pull --ff-only origin "$BRANCH"

if [[ ! -x .venv/bin/python ]]; then
  python3 -m venv .venv
fi

# shellcheck disable=SC1091
source .venv/bin/activate
python -m pip install -e ".[gui]" >/dev/null

STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
OUT_DIR="$HOME/soarm-motion-tests/sleep2-comparison-$STAMP"
mkdir -p "$OUT_DIR"
TRACE="$OUT_DIR/sleep2-comparison.jsonl"

echo
echo "SO-ARM101 Sleep vs sleep2 comparison"
echo "Exact commit: $(git rev-parse HEAD)"
echo "Speed: $SPEED_DEG_S deg/s"
echo "Acceleration: $ACCEL_DEG_S2 deg/s^2"
echo
echo "The CURRENT physical arm pose will be captured as the named pose 'sleep2'."
echo "Then:"
echo "  A: RIGHT -> canonical Sleep"
echo "  B: RIGHT -> sleep2"
echo
echo "Keep physical power accessible and the workspace clear."

status=0
python examples/sleep2_comparison.py \
  --speed-deg-s "$SPEED_DEG_S" \
  --acceleration-deg-s2 "$ACCEL_DEG_S2" \
  --output "$TRACE" || status=$?

cat > "$OUT_DIR/README.txt" <<EOF
SO-ARM101 Sleep vs sleep2 comparison

Git SHA: $(git rev-parse HEAD)
Speed: $SPEED_DEG_S deg/s
Acceleration: $ACCEL_DEG_S2 deg/s^2
Exit status: $status

The pose present when the Python program connected was captured before any
arm motion or torque-latch command, validated against calibrated joint limits,
and saved in the robot's PoseLibrary as:

  sleep2

A:
  RIGHT -> canonical Sleep.

B:
  RIGHT -> recorded sleep2.

sleep2 uses the same narrow Sleep-family treatment of the generic coarse
self-clearance heuristic. Its joint-space route is still preflighted for
calibrated joint limits, floor, TCP reach, and base keepout; normal streamed
runtime motion guards remain active.

Artifacts in this folder may include:
  sleep2.json
  sleep2-comparison.jsonl
  sleep2-comparison.summary.json
EOF

ARCHIVE="$OUT_DIR.tar.gz"
tar -C "$(dirname "$OUT_DIR")" -czf "$ARCHIVE" "$(basename "$OUT_DIR")"

echo
echo "========================================================================"
echo "SLEEP VS SLEEP2 COMPARISON COMPLETE"
echo "========================================================================"
echo "Folder: $OUT_DIR"
[[ -f "$OUT_DIR/sleep2.json" ]] && echo "Captured pose: $OUT_DIR/sleep2.json"
[[ -f "$TRACE" ]] && echo "Trace: $TRACE"
[[ -f "$OUT_DIR/sleep2-comparison.summary.json" ]] && \
  echo "Summary: $OUT_DIR/sleep2-comparison.summary.json"
echo "Archive: $ARCHIVE"
echo "Follower remains torque-held."
if (( status != 0 )); then
  echo "The run stopped early; available evidence was still packaged."
fi
exit "$status"
