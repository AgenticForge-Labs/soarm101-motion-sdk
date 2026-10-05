#!/usr/bin/env bash
set -euo pipefail

BRANCH="fix/quantized-slow-tail"
MODE="${1:-streamed}"
SPEED_DEG_S="${2:-24}"
ACCEL_DEG_S2="${3:-150}"

if [[ "$MODE" != "streamed" ]]; then
  echo "usage: $0 [streamed] [speed_deg_s] [accel_deg_s2]" >&2
  exit 2
fi

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
OUT_DIR="$HOME/soarm-motion-tests/sleep-geometry-comparison-$STAMP"
mkdir -p "$OUT_DIR"
TRACE="$OUT_DIR/sleep-geometry.jsonl"

echo
echo "SO-ARM101 Sleep geometry comparison"
echo "Exact commit: $(git rev-parse HEAD)"
echo "Mode: $MODE"
echo "Speed: $SPEED_DEG_S deg/s"
echo "Acceleration: $ACCEL_DEG_S2 deg/s^2"
echo
echo "Teach: canonical Sleep -> relax wrist_flex only -> hand-place wrist -> ENTER -> relatch/save"
echo "A: RIGHT -> canonical Sleep"
echo "B: RIGHT -> same folded Sleep arm geometry with your taught wrist angle"
echo "C: RIGHT -> taught-wrist Sleep -> fold only wrist_flex into canonical Sleep"
echo
echo "Keep physical power accessible and the workspace clear."

status=0
python examples/sleep_geometry_comparison.py   --execution-mode "$MODE"   --speed-deg-s "$SPEED_DEG_S"   --acceleration-deg-s2 "$ACCEL_DEG_S2"   --output "$TRACE" || status=$?

cat > "$OUT_DIR/README.txt" <<EOF
SO-ARM101 Sleep geometry comparison

Git SHA: $(git rev-parse HEAD)
Mode: $MODE
Speed: $SPEED_DEG_S deg/s
Acceleration: $ACCEL_DEG_S2 deg/s^2
Exit status: $status

Teach:
  The arm first enters canonical Sleep and holds. Only wrist_flex is relaxed.
  The operator hand-places that wrist and presses ENTER. The measured wrist must
  remain inside the executable calibrated range; non-wrist joint drift is checked
  before wrist_flex is relatched. The complete taught pose is saved locally as
  motion_test_sleep_wrist and in taught-sleep-wrist.json.

A direct:
  RIGHT -> canonical Sleep.

B taught_wrist:
  RIGHT -> the same canonical folded arm geometry, but using the operator-taught
  wrist_flex angle.

C staged:
  RIGHT -> taught-wrist Sleep, then only wrist_flex moves into canonical Sleep.

B/C use the same narrow Sleep-family exception for the generic coarse
self-clearance heuristic. Before motion, the full joint path is checked for
calibrated joint limits, floor, reach, and base keepout with self-clearance alone
disabled. Runtime rate/acceleration/following-error/effort/fault/communication/
settle guards remain active. This diagnostic is intentionally streamed-only.
EOF

ARCHIVE="$OUT_DIR.tar.gz"
tar -C "$(dirname "$OUT_DIR")" -czf "$ARCHIVE" "$(basename "$OUT_DIR")"

echo
echo "========================================================================"
echo "SLEEP GEOMETRY COMPARISON COMPLETE"
echo "========================================================================"
echo "Folder: $OUT_DIR"
[[ -f "$TRACE" ]] && echo "Trace: $TRACE"
echo "Archive: $ARCHIVE"
echo "Follower remains torque-held."
if (( status != 0 )); then
  echo "The run stopped early; the partial evidence was still packaged."
fi
exit "$status"
