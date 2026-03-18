#!/usr/bin/env bash
set -euo pipefail

PARK_PID_FILE="/tmp/rplidar_park.pid"

if [[ -f "$PARK_PID_FILE" ]]; then
  pid="$(cat "$PARK_PID_FILE" || true)"
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    kill "$pid" || true
    wait "$pid" 2>/dev/null || true
    echo "stopped park daemon pid=$pid"
  else
    echo "park daemon pid file found but process is not running"
  fi
  rm -f "$PARK_PID_FILE"
else
  echo "no park daemon pid file"
fi
