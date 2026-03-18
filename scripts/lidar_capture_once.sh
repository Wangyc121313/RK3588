#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PARK_PID_FILE="/tmp/rplidar_park.pid"

PORT="${1:-/dev/ttyUSB0}"
BAUD="${2:-115200}"
CAPTURE_SEC="${3:-3}"
IDLE_HOLD_SEC="${4:-1}"
PARK_HOLD_SEC="${5:-3600}"

stop_park_daemon() {
  if [[ -f "$PARK_PID_FILE" ]]; then
    local pid
    pid="$(cat "$PARK_PID_FILE" || true)"
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill "$pid" || true
      wait "$pid" 2>/dev/null || true
    fi
    rm -f "$PARK_PID_FILE"
  fi
}

if [[ ! -x "$ROOT_DIR/build/rplidar_timed_demo" || ! -x "$ROOT_DIR/build/rplidar_motor_ctl" ]]; then
  echo "error: required binaries not found; build first with cmake --build build -j4" >&2
  exit 1
fi

# Release previous parking holder so capture can run.
stop_park_daemon

"$ROOT_DIR/build/rplidar_timed_demo" "$PORT" "$BAUD" "$CAPTURE_SEC" "$IDLE_HOLD_SEC"

# Re-assert and hold motor stop in background after capture process exits.
nohup "$ROOT_DIR/build/rplidar_motor_ctl" stop "$PORT" "$BAUD" "$PARK_HOLD_SEC" \
  >/tmp/rplidar_park.log 2>&1 &

echo $! > "$PARK_PID_FILE"
echo "capture done; motor park daemon started with pid $(cat "$PARK_PID_FILE")"
