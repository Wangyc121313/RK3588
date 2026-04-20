#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

CAMERA_DEVICE="${1:-/dev/video0}"
CAMERA_WIDTH="${2:-640}"
CAMERA_HEIGHT="${3:-480}"
RUN_SECONDS="${4:-60}"
OUTPUT_PATH="${5:-/tmp/rk3588_pseudo_labels.jsonl}"
TELEMETRY_PATH="${6:-/tmp/rk3588_telemetry.jsonl}"

export RK3588_PSEUDO_LABEL_PATH="$OUTPUT_PATH"
export RK3588_PSEUDO_LABEL_MAX_LINES="${RK3588_PSEUDO_LABEL_MAX_LINES:-5000}"
export RK3588_PSEUDO_LABEL_SEQUENCE_ID="${RK3588_PSEUDO_LABEL_SEQUENCE_ID:-capture_$(date +%Y%m%d_%H%M%S)}"
export RK3588_TELEMETRY_PATH="$TELEMETRY_PATH"

echo "[capture] output=$RK3588_PSEUDO_LABEL_PATH"
echo "[capture] max_lines=$RK3588_PSEUDO_LABEL_MAX_LINES"
echo "[capture] sequence_id=$RK3588_PSEUDO_LABEL_SEQUENCE_ID"
echo "[capture] telemetry=$RK3588_TELEMETRY_PATH"
echo "[capture] run_seconds=$RUN_SECONDS"

if [[ ! -x ./build/perception_app ]]; then
  echo "error: ./build/perception_app not found, please build first"
  exit 2
fi

./build/perception_app "$CAMERA_DEVICE" "$CAMERA_WIDTH" "$CAMERA_HEIGHT" "$RUN_SECONDS"

python3 tools/diagnostics/validate_pseudo_labels.py --glob "${OUTPUT_PATH}*"
python3 tools/diagnostics/analyze_tracking_metrics.py --glob "${OUTPUT_PATH}*"
