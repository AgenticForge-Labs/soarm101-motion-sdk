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
OUT_DIR="$HOME/soarm-motion-tests/sleep-vs-sleep-up-$STAMP"
mkdir -p "$OUT_DIR"
TRACE="$OUT_DIR/sleep-vs-sleep-up.jsonl"

echo
echo "SO-ARM101 default Sleep vs sleep_up comparison"
echo "Exact commit: $(git rev-parse HEAD)"
echo "Speed: $SPEED_DEG_S deg/s"
echo "Acceleration: $ACCEL_DEG_S2 deg/s^2"
echo
echo "A: RIGHT -> new default Sleep (wrist_flex at 75% of executable range)"
echo "B: RIGHT -> historical sleep_up (wrist_flex at lower executable limit)"
echo
echo "Keep physical power accessible and the workspace clear."

status=0
python examples/sleep_posture_comparison.py   --speed-deg-s "$SPEED_DEG_S"   --acceleration-deg-s2 "$ACCEL_DEG_S2"   --output "$TRACE" || status=$?

cat > "$OUT_DIR/README.txt" <<EOF
SO-ARM101 default Sleep vs sleep_up comparison

Git SHA: $(git rev-parse HEAD)
Speed: $SPEED_DEG_S deg/s
Acceleration: $ACCEL_DEG_S2 deg/s^2
Exit status: $status

A:
  RIGHT -> default Sleep.
  wrist_flex = lower + 0.75 * (upper - lower).

B:
  RIGHT -> sleep_up.
  wrist_flex = lower executable limit.

All other Sleep joint selectors are identical and calibration-relative.

Both conditions use the dedicated Sleep-family motion primitives and therefore
retain calibrated joint/tool limits and normal runtime motion guards while using
the existing narrow folded-posture exception to the generic coarse
self-clearance heuristic.

Artifacts:
  sleep-vs-sleep-up.jsonl
  sleep-vs-sleep-up.summary.json
EOF

ARCHIVE="$OUT_DIR.tar.gz"
tar -C "$(dirname "$OUT_DIR")" -czf "$ARCHIVE" "$(basename "$OUT_DIR")"

echo
echo "========================================================================"
echo "DEFAULT SLEEP VS SLEEP_UP COMPARISON COMPLETE"
echo "========================================================================"
echo "Folder: $OUT_DIR"
[[ -f "$TRACE" ]] && echo "Trace: $TRACE"
[[ -f "$OUT_DIR/sleep-vs-sleep-up.summary.json" ]] &&   echo "Summary: $OUT_DIR/sleep-vs-sleep-up.summary.json"
echo "Archive: $ARCHIVE"
echo "Follower remains torque-held."
if (( status != 0 )); then
  echo "The run stopped early; available evidence was still packaged."
fi
exit "$status"
