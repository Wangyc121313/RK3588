#!/usr/bin/env bash
set -euo pipefail

DEVICE="${1:-/dev/video0}"
FPS="${2:-}"

if [[ ! -e "$DEVICE" ]]; then
    echo "camera device not found: $DEVICE" >&2
    exit 1
fi

if ! command -v v4l2-ctl >/dev/null 2>&1; then
    echo "v4l2-ctl is required (install v4l-utils)" >&2
    exit 1
fi

if ! command -v ffplay >/dev/null 2>&1; then
    echo "ffplay is required (install ffmpeg package with ffplay)" >&2
    exit 1
fi

FMT_INFO="$(v4l2-ctl -d "$DEVICE" --get-fmt-video 2>/dev/null || true)"
if [[ -z "$FMT_INFO" ]]; then
    echo "failed to query camera format from $DEVICE" >&2
    exit 1
fi

WIDTH="$(echo "$FMT_INFO" | sed -n 's/.*Width\/Height[[:space:]]*:[[:space:]]*\([0-9]\+\)\/\([0-9]\+\).*/\1/p')"
HEIGHT="$(echo "$FMT_INFO" | sed -n 's/.*Width\/Height[[:space:]]*:[[:space:]]*\([0-9]\+\)\/\([0-9]\+\).*/\2/p')"
PIXEL="$(echo "$FMT_INFO" | sed -n "s/.*Pixel Format[[:space:]]*:[[:space:]]*'\([^']\+\)'.*/\1/p")"

if [[ -z "$WIDTH" || -z "$HEIGHT" || -z "$PIXEL" ]]; then
    echo "failed to parse camera format: $FMT_INFO" >&2
    exit 1
fi

if [[ -z "$FPS" ]]; then
    PARM_INFO="$(v4l2-ctl -d "$DEVICE" --get-parm 2>/dev/null || true)"
    FPS="$(echo "$PARM_INFO" | sed -n 's/.*([0-9]\+\.[0-9]\+ fps).*/\1/p' | head -n 1)"
    FPS="${FPS:-30}"
fi

FFMPEG_PIXEL=""
case "$PIXEL" in
    YUYV)
        FFMPEG_PIXEL="yuyv422"
        ;;
    NV12)
        FFMPEG_PIXEL="nv12"
        ;;
    MJPG)
        FFMPEG_PIXEL="mjpeg"
        ;;
    *)
        FFMPEG_PIXEL=""
        ;;
esac

echo "[preview] device=$DEVICE size=${WIDTH}x${HEIGHT} pixel=$PIXEL fps=$FPS"

CMD=(ffplay
    -hide_banner
    -loglevel warning
    -fflags nobuffer
    -flags low_delay
    -framedrop
    -f v4l2
    -framerate "$FPS"
    -video_size "${WIDTH}x${HEIGHT}")

if [[ -n "$FFMPEG_PIXEL" ]]; then
    CMD+=( -input_format "$FFMPEG_PIXEL" )
fi

CMD+=( "$DEVICE" )

exec "${CMD[@]}"
