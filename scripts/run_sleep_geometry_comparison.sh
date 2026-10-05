#!/usr/bin/env bash
set -euo pipefail

BRANCH="fix/quantized-slow-tail"
MODE="${1:-streamed}"
SPEED_DEG_S="${2:-24}"
ACCEL_DEG_S2="${3:-150}"

if [[ "$MODE" != "streamed" && "$MODE" != "final_target" ]]; then
  echo "usage: $0 [streamed|final_target] [speed_deg_s] [accel_deg_s2]" >&2
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
echo "A: RIGHT -> canonical Sleep"
echo "B: RIGHT -> deepest strict-workspace-valid neutral-wrist pre-Sleep"
echo "C: RIGHT -> validated open pre-Sleep -> canonical Sleep"
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

A direct:
  RIGHT -> canonical Sleep.

B open_pre_sleep:
  RIGHT -> the deepest target found along the neutral-wrist fold toward Sleep
  whose entire joint path passes the strict workspace envelope with an extra
  2 mm coarse self-clearance reserve.

C staged_sleep:
  RIGHT -> that validated open pre-Sleep -> canonical Sleep. This tests whether
  the severe rocking appears only when entering the final deep fold.

The open pre-Sleep uses the ordinary full workspace validator. Canonical Sleep
retains its existing deliberate coarse-workspace exception and all other motion
guards.
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
