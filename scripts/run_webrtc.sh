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

PUBLISH_MODE="${PUBLISH_MODE:-webrtc}"
WEBRTC_URL="${WEBRTC_URL:-rtc://127.0.0.1:8000/live/camera}"
DEBUG_UI_HOST="${DEBUG_UI_HOST:-0.0.0.0}"
DEBUG_UI_PORT="${DEBUG_UI_PORT:-8090}"
DEBUG_UI_ZLM_HOST="${DEBUG_UI_ZLM_HOST:-127.0.0.1}"
START_DEBUG_UI="${START_DEBUG_UI:-1}"
DEBUG_UI_LOG_PATH="${DEBUG_UI_LOG_PATH:-/tmp/rk3588_webrtc_debug_ui.log}"
export RK3588_TELEMETRY_PATH="${RK3588_TELEMETRY_PATH:-/tmp/rk3588_perception_telemetry.jsonl}"
export RK3588_TELEMETRY_INTERVAL_MS="${RK3588_TELEMETRY_INTERVAL_MS:-500}"

export RK3588_WEBRTC_RTC_PORT="${RK3588_WEBRTC_RTC_PORT:-8000}"
export RK3588_WEBRTC_ICE_PORT="${RK3588_WEBRTC_ICE_PORT:-8001}"
export RK3588_WEBRTC_SIGNALING_PORT="${RK3588_WEBRTC_SIGNALING_PORT:-10000}"
export RK3588_WEBRTC_HTTP_PORT="${RK3588_WEBRTC_HTTP_PORT:-8080}"

check_port_free() {
	local port="$1"
	if ss -ltnu | awk '{print $5}' | grep -Eq "[:.]${port}$"; then
		echo "Port ${port} is already in use."
		ss -ltnup | grep -E ":${port}(\\s|$)" || true
		return 1
	fi
	return 0
}

echo "Checking WebRTC ports..."
check_port_free "$RK3588_WEBRTC_RTC_PORT"
check_port_free "$RK3588_WEBRTC_ICE_PORT"
check_port_free "$RK3588_WEBRTC_SIGNALING_PORT"
check_port_free "$RK3588_WEBRTC_HTTP_PORT"
if [[ "$START_DEBUG_UI" == "1" ]]; then
	check_port_free "$DEBUG_UI_PORT"
fi

DEVICE="$(resolve_camera_device "$DEVICE")"

LOCAL_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
WEBRTC_PLAYER_PATH="/webrtc/index.html?app=live&stream=camera&type=play"
ZLM_LOCAL_URL="http://127.0.0.1:${RK3588_WEBRTC_HTTP_PORT}${WEBRTC_PLAYER_PATH}"
DEBUG_UI_LOCAL_URL="http://127.0.0.1:${DEBUG_UI_PORT}/?app=live&stream=camera"
ZLM_LAN_URL=""
DEBUG_UI_LAN_URL=""
if [[ -n "$LOCAL_IP" ]]; then
	ZLM_LAN_URL="http://${LOCAL_IP}:${RK3588_WEBRTC_HTTP_PORT}${WEBRTC_PLAYER_PATH}"
	DEBUG_UI_LAN_URL="http://${LOCAL_IP}:${DEBUG_UI_PORT}/?app=live&stream=camera"
fi

echo "Starting perception_app in mode=${PUBLISH_MODE}"
echo "Camera device: ${DEVICE}"
echo "Telemetry path: ${RK3588_TELEMETRY_PATH}"
echo "Open in browser (local): ${ZLM_LOCAL_URL}"
if [[ -n "$ZLM_LAN_URL" ]]; then
	echo "Open in browser (LAN):   ${ZLM_LAN_URL}"
fi
if [[ "$START_DEBUG_UI" == "1" ]]; then
	echo "Debug UI (local):        ${DEBUG_UI_LOCAL_URL}"
	if [[ -n "$DEBUG_UI_LAN_URL" ]]; then
		echo "Debug UI (LAN):          ${DEBUG_UI_LAN_URL}"
	fi
	nohup python3 "$ROOT_DIR/tools/webrtc_debug_ui/server.py" \
		--host "$DEBUG_UI_HOST" \
		--port "$DEBUG_UI_PORT" \
		--zlm-host "$DEBUG_UI_ZLM_HOST" \
		--zlm-http-port "$RK3588_WEBRTC_HTTP_PORT" \
		--telemetry-path "$RK3588_TELEMETRY_PATH" \
		--default-app live \
		--default-stream camera \
		>"$DEBUG_UI_LOG_PATH" 2>&1 &
	DEBUG_UI_PID=$!
	echo "Debug UI log: ${DEBUG_UI_LOG_PATH}"
	cleanup_debug_ui() {
		kill "$DEBUG_UI_PID" 2>/dev/null || true
	}
	trap cleanup_debug_ui EXIT INT TERM
fi

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

"${CMD[@]}"
APP_EXIT_CODE=$?

if [[ -n "${DEBUG_UI_PID:-}" ]]; then
	kill "$DEBUG_UI_PID" 2>/dev/null || true
	wait "$DEBUG_UI_PID" 2>/dev/null || true
fi

exit "$APP_EXIT_CODE"
