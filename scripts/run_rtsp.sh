#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

. "$ROOT_DIR/scripts/camera_device_utils.sh"

APP_BIN="${APP_BIN:-$ROOT_DIR/build/perception_app}"
if [[ ! -x "$APP_BIN" ]]; then
	echo "perception_app not found: $APP_BIN"
	echo "Build first: cmake -S . -B build && cmake --build build -j4 --target perception_app"
	exit 1
fi

DEVICE="${DEVICE:-auto}"
WIDTH="${WIDTH:-640}"
HEIGHT="${HEIGHT:-480}"
RUN_SECONDS="${RUN_SECONDS:-0}"
MODEL_PATH="${MODEL_PATH:-models/yolov8n.rknn}"
MODEL_W="${MODEL_W:-640}"
MODEL_H="${MODEL_H:-640}"
LABELS_PATH="${LABELS_PATH:-models/coco_80_labels_lists.txt}"
RTSP_URL="${RTSP_URL:-rtsp://127.0.0.1:8554/live/camera}"
FPS="${FPS:-30}"
DUMP_H264_PATH="${DUMP_H264_PATH:-}"
INFER_EVERY_N="${INFER_EVERY_N:-5}"
LIDAR_PORT="${LIDAR_PORT:-/dev/ttyUSB0}"
LIDAR_BAUD="${LIDAR_BAUD:-115200}"
export RK3588_CAMERA_FOV_DEG="${RK3588_CAMERA_FOV_DEG:-55}"
LIDAR_OFFSET_DEG="${LIDAR_OFFSET_DEG:-11.7}"
LIDAR_FOV_DEG="${LIDAR_FOV_DEG:-55}"
LIDAR_WINDOW_HALF_DEG="${LIDAR_WINDOW_HALF_DEG:-2.5}"
LIDAR_MIN_DIST_M="${LIDAR_MIN_DIST_M:-0.15}"
LIDAR_MAX_DIST_M="${LIDAR_MAX_DIST_M:-8.0}"
LIDAR_MAX_AGE_MS="${LIDAR_MAX_AGE_MS:-120}"

PUBLISH_MODE="${PUBLISH_MODE:-rtsp}"
WEBRTC_URL="${WEBRTC_URL:-rtc://127.0.0.1:8000/live/camera}"

RTSP_PORT="${RTSP_PORT:-8554}"
if ss -ltnu | awk '{print $5}' | grep -Eq "[:.]${RTSP_PORT}$"; then
	echo "RTSP port ${RTSP_PORT} is already in use."
	ss -ltnup | grep -E ":${RTSP_PORT}(\\s|$)" || true
	echo "If this is an old process, stop it first (example: sudo pkill -f perception_app)."
	exit 1
fi

DEVICE="$(resolve_camera_device "$DEVICE")"

echo "Starting perception_app in mode=${PUBLISH_MODE}"
echo "Camera device: ${DEVICE}"
echo "RTSP URL: ${RTSP_URL}"
echo "VLC preview: vlc ${RTSP_URL}"

CMD=(
	"$APP_BIN"
	"$DEVICE"
	"$WIDTH"
	"$HEIGHT"
	"$RUN_SECONDS"
	"$MODEL_PATH"
	"$MODEL_W"
	"$MODEL_H"
	"$LABELS_PATH"
	"$RTSP_URL"
	"$FPS"
	"$DUMP_H264_PATH"
	"$INFER_EVERY_N"
	"$LIDAR_PORT"
	"$LIDAR_BAUD"
	"$LIDAR_OFFSET_DEG"
	"$LIDAR_FOV_DEG"
	"$LIDAR_WINDOW_HALF_DEG"
	"$LIDAR_MIN_DIST_M"
	"$LIDAR_MAX_DIST_M"
	"$LIDAR_MAX_AGE_MS"
	"$PUBLISH_MODE"
	"$WEBRTC_URL"
)

printf 'Command: '
printf '%q ' "${CMD[@]}"
printf '\n'

exec "${CMD[@]}"
