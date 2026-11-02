#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-both}"
SPEED_DEG_S="${2:-8}"
ACCEL_DEG_S2="${3:-25}"
if [[ "$MODE" != "both" && "$MODE" != "minimum_jerk" && "$MODE" != "cosine_cruise" ]]; then
  echo "usage: $0 [both|minimum_jerk|cosine_cruise] [speed_deg_s] [accel_deg_s2]" >&2
  exit 2
fi

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

echo "SO-ARM101 side-by-side joint trajectory profile comparison (minimum_jerk vs cosine_cruise)"
echo "Repository: $ROOT"
echo "Branch: $(git rev-parse --abbrev-ref HEAD)"
echo "Commit: $(git rev-parse HEAD)"

if [[ ! -x .venv/bin/python ]]; then
  python3 -m venv .venv
fi

# shellcheck disable=SC1091
source .venv/bin/activate
if ! python -c "import soarm101_motion" >/dev/null 2>&1; then
  python -m pip install --no-build-isolation -e . >/dev/null
fi

python - "$SPEED_DEG_S" "$ACCEL_DEG_S2" <<'PY'
import math
import sys

speed = float(sys.argv[1])
accel = float(sys.argv[2])
if not math.isfinite(speed) or speed <= 0:
    raise SystemExit("speed_deg_s must be positive and finite")
if not math.isfinite(accel) or accel <= 0:
    raise SystemExit("accel_deg_s2 must be positive and finite")
PY

SIM_FLAG=()
PAUSE_S="0.5"
if [[ "${SOARM_SIMULATION:-0}" == "1" ]]; then
  SIM_FLAG=(--simulation)
  PAUSE_S="0"
fi

STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
OUT_DIR="${SOARM_COMPARISON_OUT_DIR:-$HOME/soarm-motion-tests/joint-profile-comparison-$STAMP}"
mkdir -p "$OUT_DIR"

echo
echo "Both runs use:"
echo "  route: Sleep -> Overhead -> Left -> Right -> Sleep"
echo "  execution_mode: streamed (0/254 inner-loop servo tracking)"
echo "  requested joint speed: $SPEED_DEG_S deg/s"
echo "  requested joint acceleration: $ACCEL_DEG_S2 deg/s^2"
echo "  planning cadence: 50 Hz"
echo
echo "Run A = minimum_jerk: legacy synchronized quintic baseline (1.875x peak/mean speed ratio)."
echo "Run B = cosine_cruise: half-cosine accel/decel ramps + constant-speed cruise + calibrated"
echo "        terminal duplicate-encoder tail coalescing."
echo
if [[ "${SOARM_SIMULATION:-0}" != "1" ]]; then
  echo "Keep physical power accessible and the workspace clear."
fi

MIN_JERK_STATUS="not-run"
COSINE_STATUS="not-run"
OVERALL_STATUS=0

if [[ "$MODE" == "both" || "$MODE" == "minimum_jerk" ]]; then
  if [[ "${SOARM_SIMULATION:-0}" != "1" ]]; then
    read -r -p "Press ENTER to start minimum_jerk motion, or Ctrl-C to stop: "
  fi
  mj_status=0
  python examples/motion_quality_trace.py \
    "${SIM_FLAG[@]}" \
    --execution-mode streamed \
    --profile-mode minimum_jerk \
    --speed-deg-s "$SPEED_DEG_S" \
    --acceleration-deg-s2 "$ACCEL_DEG_S2" \
    --command-frequency-hz 50 \
    --pause-s "$PAUSE_S" \
    --output "$OUT_DIR/minimum-jerk.jsonl" || mj_status=$?
  MIN_JERK_STATUS="$mj_status"
  if (( mj_status == 0 )); then
    echo
    echo "minimum_jerk run complete. The arm should be holding in Sleep."
  else
    echo
    echo "minimum_jerk run stopped with status $mj_status; preserving evidence."
    OVERALL_STATUS="$mj_status"
  fi
fi

if [[ "$MODE" == "both" && "$MIN_JERK_STATUS" != "0" ]]; then
  echo "Skipping cosine_cruise because the minimum_jerk baseline did not complete cleanly."
elif [[ "$MODE" == "both" || "$MODE" == "cosine_cruise" ]]; then
  if [[ "${SOARM_SIMULATION:-0}" != "1" ]]; then
    read -r -p "Press ENTER to start cosine_cruise motion, or Ctrl-C to stop: "
  fi
  cc_status=0
  python examples/motion_quality_trace.py \
    "${SIM_FLAG[@]}" \
    --execution-mode streamed \
    --profile-mode cosine_cruise \
    --speed-deg-s "$SPEED_DEG_S" \
    --acceleration-deg-s2 "$ACCEL_DEG_S2" \
    --command-frequency-hz 50 \
    --pause-s "$PAUSE_S" \
    --output "$OUT_DIR/cosine-cruise.jsonl" || cc_status=$?
  COSINE_STATUS="$cc_status"
  if (( cc_status == 0 )); then
    echo
    echo "cosine_cruise run complete. The arm should be holding in Sleep."
  else
    echo
    echo "cosine_cruise run stopped with status $cc_status; preserving evidence."
    OVERALL_STATUS="$cc_status"
  fi
fi

TRACE_ARGS=()
[[ -f "$OUT_DIR/minimum-jerk.jsonl" ]] && TRACE_ARGS+=(--trace "minimum_jerk=$OUT_DIR/minimum-jerk.jsonl")
[[ -f "$OUT_DIR/cosine-cruise.jsonl" ]] && TRACE_ARGS+=(--trace "cosine_cruise=$OUT_DIR/cosine-cruise.jsonl")

if (( ${#TRACE_ARGS[@]} > 0 )); then
  GRIPPER_ARGS=()
  while IFS= read -r token; do
    [[ -n "$token" ]] && GRIPPER_ARGS+=("$token")
  done < <(python - <<'PY'
import json
from soarm101_motion.calibration import default_calibration_path
from soarm101_motion.workstation import WorkstationProfileStore

try:
    profile = WorkstationProfileStore().load()
    robot_id = profile.follower.robot_id or "so101"
    cal_path = default_calibration_path(robot_id)
    if cal_path.is_file():
        data = json.loads(cal_path.read_text(encoding="utf-8"))
        gripper = data.get("gripper") or data.get("motors", {}).get("gripper") or {}
        rmin = gripper.get("range_min")
        rmax = gripper.get("range_max")
        dmode = gripper.get("drive_mode", 0)
        if isinstance(rmin, int) and isinstance(rmax, int) and 0 <= rmin < rmax <= 4095:
            print("--gripper-raw-range")
            print(str(rmin))
            print(str(rmax))
            if int(dmode) == 1:
                print("--gripper-reversed")
except Exception:
    pass
PY
  )

  echo
  echo "========================================================================"
  echo "WOBBLE TRACE ANALYSIS"
  echo "========================================================================"
  python scripts/analyze_wobble_traces.py \
    "${TRACE_ARGS[@]}" \
    "${GRIPPER_ARGS[@]}" \
    --out-json "$OUT_DIR/wobble-comparison.json" \
    --out-csv "$OUT_DIR/wobble-comparison.csv" | tee "$OUT_DIR/wobble-comparison.txt"
fi

cat > "$OUT_DIR/README.txt" <<EOF
SO-ARM101 joint trajectory profile comparison

Git SHA: $(git rev-parse HEAD)
Simulation: ${SOARM_SIMULATION:-0}

Requested mode: $MODE
minimum_jerk exit status: $MIN_JERK_STATUS
cosine_cruise exit status: $COSINE_STATUS

Route: Sleep -> Overhead -> Left -> Right -> Sleep
Execution mode: streamed
Requested joint speed: $SPEED_DEG_S deg/s
Requested joint acceleration: $ACCEL_DEG_S2 deg/s^2
Planning cadence: 50 Hz

minimum_jerk:
  Synchronized straight-line joint interpolation using quintic progress 10u^3 - 15u^4 + 6u^5
  (duration >= max(1.875 * delta / speed, sqrt(5.7735 * delta / acceleration))).

cosine_cruise:
  Synchronized straight-line joint interpolation using symmetric half-cosine acceleration and
  deceleration ramps with a constant-speed cruise section (matching Cartesian move_linear timing
  semantics) plus guard-verified coalescing of trailing duplicate-encoder samples on calibrated
  hardware.
EOF

ARCHIVE="$OUT_DIR.tar.gz"
tar -C "$(dirname "$OUT_DIR")" -czf "$ARCHIVE" "$(basename "$OUT_DIR")"

echo
echo "========================================================================"
echo "COMPARISON COMPLETE"
echo "========================================================================"
echo "Folder: $OUT_DIR"
[[ -f "$OUT_DIR/minimum-jerk.jsonl" ]] && echo "minimum_jerk trace: $OUT_DIR/minimum-jerk.jsonl"
[[ -f "$OUT_DIR/cosine-cruise.jsonl" ]] && echo "cosine_cruise trace: $OUT_DIR/cosine-cruise.jsonl"
[[ -f "$OUT_DIR/wobble-comparison.json" ]] && echo "Analysis JSON: $OUT_DIR/wobble-comparison.json"
[[ -f "$OUT_DIR/wobble-comparison.csv" ]] && echo "Analysis CSV: $OUT_DIR/wobble-comparison.csv"
echo "Archive: $ARCHIVE"
echo
if [[ "${SOARM_SIMULATION:-0}" != "1" ]]; then
  echo "Follower remains torque-held."
fi
if (( OVERALL_STATUS != 0 )); then
  echo "One or more requested runs stopped early; inspect or archive $OUT_DIR for analysis."
fi
exit "$OVERALL_STATUS"
