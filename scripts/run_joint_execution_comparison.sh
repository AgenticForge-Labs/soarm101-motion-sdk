#!/usr/bin/env bash
set -euo pipefail

BRANCH="fix/quantized-slow-tail"
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
read -r -p "Press ENTER to start RUN A (streamed), or Ctrl-C to stop: "

python examples/motion_quality_trace.py   --execution-mode streamed   --speed-deg-s 8   --acceleration-deg-s2 25   --command-frequency-hz 50   --pause-s 0.5   --output "$OUT_DIR/streamed.jsonl"

echo
echo "RUN A complete. The arm should be holding in Sleep."
read -r -p "Press ENTER to start RUN B (final_target), or Ctrl-C to stop: "

python examples/motion_quality_trace.py   --execution-mode final_target   --speed-deg-s 8   --acceleration-deg-s2 25   --command-frequency-hz 50   --pause-s 0.5   --output "$OUT_DIR/final-target.jsonl"

cat > "$OUT_DIR/README.txt" <<EOF
SO-ARM101 joint execution comparison

Git SHA: $(git rev-parse HEAD)

Run A: streamed
Run B: final_target

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
  joint-space corridor/coordination error, timeout/cancellation, and final settle.

Cartesian move_linear is not part of this comparison and remains host-streamed.
EOF

ARCHIVE="$OUT_DIR.tar.gz"
tar -C "$(dirname "$OUT_DIR")" -czf "$ARCHIVE" "$(basename "$OUT_DIR")"

echo
echo "========================================================================"
echo "COMPARISON COMPLETE"
echo "========================================================================"
echo "Folder: $OUT_DIR"
echo "Streamed trace: $OUT_DIR/streamed.jsonl"
echo "Final-target trace: $OUT_DIR/final-target.jsonl"
echo "Archive: $ARCHIVE"
echo
echo "Follower remains torque-held."
