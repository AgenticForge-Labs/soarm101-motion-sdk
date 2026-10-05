#!/usr/bin/env bash
set -euo pipefail

BRANCH="fix/quantized-slow-tail"
MODE="${1:-both}"
if [[ "$MODE" != "both" && "$MODE" != "streamed" && "$MODE" != "final_target" ]]; then
  echo "usage: $0 [both|streamed|final_target]" >&2
  exit 2
fi

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

echo "SO-ARM101 streamed vs final-target joint execution comparison"
echo "Repository: $ROOT"
echo "Updating branch: $BRANCH"

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
OUT_DIR="$HOME/soarm-motion-tests/joint-execution-comparison-$STAMP"
mkdir -p "$OUT_DIR"

echo
echo "Exact commit:"
git rev-parse HEAD
echo
echo "Both runs use:"
echo "  route: Sleep -> Overhead -> Left -> Right -> Sleep"
echo "  requested joint speed: 8 deg/s"
echo "  requested joint acceleration: 25 deg/s^2"
echo "  planning cadence: 50 Hz"
echo
echo "Run A = streamed: validated host trajectory sends intermediate joint targets."
echo "Run B = final_target: same validated plan, one synchronized endpoint write per leg,"
echo "        with guarded monitoring until settle."
echo
echo "Keep physical power accessible and the workspace clear."

STREAMED_STATUS="not-run"
FINAL_TARGET_STATUS="not-run"
OVERALL_STATUS=0

if [[ "$MODE" == "both" || "$MODE" == "streamed" ]]; then
  read -r -p "Press ENTER to start streamed motion, or Ctrl-C to stop: "
  streamed_status=0
  python examples/motion_quality_trace.py \
    --execution-mode streamed \
    --speed-deg-s 8 \
    --acceleration-deg-s2 25 \
    --command-frequency-hz 50 \
    --pause-s 0.5 \
    --output "$OUT_DIR/streamed.jsonl" || streamed_status=$?
  STREAMED_STATUS="$streamed_status"
  if (( streamed_status == 0 )); then
    echo
    echo "Streamed run complete. The arm should be holding in Sleep."
  else
    echo
    echo "Streamed run stopped with status $streamed_status; preserving evidence."
    OVERALL_STATUS="$streamed_status"
  fi
fi

if [[ "$MODE" == "both" && "$STREAMED_STATUS" != "0" ]]; then
  echo "Skipping final_target because the streamed baseline did not complete cleanly."
elif [[ "$MODE" == "both" || "$MODE" == "final_target" ]]; then
  read -r -p "Press ENTER to start final_target motion, or Ctrl-C to stop: "
  final_status=0
  python examples/motion_quality_trace.py \
    --execution-mode final_target \
    --speed-deg-s 8 \
    --acceleration-deg-s2 25 \
    --command-frequency-hz 50 \
    --pause-s 0.5 \
    --output "$OUT_DIR/final-target.jsonl" || final_status=$?
  FINAL_TARGET_STATUS="$final_status"
  if (( final_status == 0 )); then
    echo
    echo "Final-target run complete. The arm should be holding in Sleep."
  else
    echo
    echo "Final-target run stopped with status $final_status; preserving evidence."
    OVERALL_STATUS="$final_status"
  fi
fi

cat > "$OUT_DIR/README.txt" <<EOF
SO-ARM101 joint execution comparison

Git SHA: $(git rev-parse HEAD)

Requested mode: $MODE
Streamed exit status: $STREAMED_STATUS
Final-target exit status: $FINAL_TARGET_STATUS

Route: Sleep -> Overhead -> Left -> Right -> Sleep
Requested joint speed: 8 deg/s
Requested joint acceleration: 25 deg/s^2
Planning cadence: 50 Hz

streamed:
  The validated host trajectory sends intermediate joint targets throughout each leg.

final_target:
  The same joint plan is built and validated, but the endpoint is written once per leg.
  Per-joint servo speed limits are derived from the validated planned duration so the
  joints are asked to arrive together. The controller monitors faults, direction,
  per-joint start-to-target corridor/overshoot, timeout/cancellation, and final settle.

Cartesian move_linear is not part of this comparison and remains host-streamed.
EOF

ARCHIVE="$OUT_DIR.tar.gz"
tar -C "$(dirname "$OUT_DIR")" -czf "$ARCHIVE" "$(basename "$OUT_DIR")"

echo
echo "========================================================================"
echo "COMPARISON COMPLETE"
echo "========================================================================"
echo "Folder: $OUT_DIR"
[[ -f "$OUT_DIR/streamed.jsonl" ]] && echo "Streamed trace: $OUT_DIR/streamed.jsonl"
[[ -f "$OUT_DIR/final-target.jsonl" ]] && echo "Final-target trace: $OUT_DIR/final-target.jsonl"
echo "Archive: $ARCHIVE"
echo
echo "Follower remains torque-held."
if (( OVERALL_STATUS != 0 )); then
  echo "One or more requested runs stopped early; upload the archive for analysis."
fi
exit "$OVERALL_STATUS"
