#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   scripts/perf_stress_suite.sh [duration_s] [rounds] [output_dir] [-- extra mpp args...]
# Example:
#   scripts/perf_stress_suite.sh 120 3 perf_runs

DURATION_S="${1:-120}"
ROUNDS="${2:-3}"
OUT_DIR="${3:-perf_runs}"
shift $(( $# >= 3 ? 3 : $# )) || true

EXTRA_ARGS=()
if [[ "${1:-}" == "--" ]]; then
    shift
    EXTRA_ARGS=("$@")
fi

mkdir -p "$OUT_DIR"
TS="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$OUT_DIR/run_$TS"
mkdir -p "$RUN_DIR"

DEFAULT_CMD=(
    ./build/mpp_encoder_demo
    /dev/video0 640 480
    "$DURATION_S"
    models/yolov8n.rknn
    640 640
    third_party/rknn_model_zoo/examples/yolov8/model/coco_80_labels_list.txt
    rtsp://0.0.0.0:8554/live/camera
    25 '' 5
    /dev/ttyUSB0 115200
    191.7 60 2.5 0.15 6.0 120
)

if [[ ${#EXTRA_ARGS[@]} -gt 0 ]]; then
    CMD=(./build/mpp_encoder_demo "${EXTRA_ARGS[@]}")
else
    CMD=("${DEFAULT_CMD[@]}")
fi

if [[ ! -x ./build/mpp_encoder_demo ]]; then
    echo "mpp_encoder_demo not found, build first:" >&2
    echo "  cmake -S . -B build && cmake --build build -j4 --target mpp_encoder_demo" >&2
    exit 1
fi

sample_runtime_stats() {
    local pid="$1"
    local csv="$2"

    echo "sec,cpu_pct,rss_mb,temp_c,rknpu_load" > "$csv"

    local clk
    clk="$(getconf CLK_TCK)"
    local pagesize
    pagesize="$(getconf PAGESIZE)"

    local prev_total=0
    local prev_proc=0
    local sec=0

    while kill -0 "$pid" 2>/dev/null; do
        local proc_stat cpu_stat
        proc_stat="$(awk '{print $14, $15, $24}' /proc/$pid/stat 2>/dev/null || true)"
        cpu_stat="$(awk '/^cpu /{print $2+$3+$4+$5+$6+$7+$8+$9+$10}' /proc/stat 2>/dev/null || true)"

        if [[ -z "$proc_stat" || -z "$cpu_stat" ]]; then
            sleep 1
            ((sec++)) || true
            continue
        fi

        local utime stime rss_pages
        read -r utime stime rss_pages <<< "$proc_stat"
        local proc_total=$((utime + stime))
        local cpu_total="$cpu_stat"

        local cpu_pct="0.00"
        if [[ "$prev_total" -gt 0 ]]; then
            local d_total=$((cpu_total - prev_total))
            local d_proc=$((proc_total - prev_proc))
            if [[ "$d_total" -gt 0 ]]; then
                cpu_pct="$(awk -v a="$d_proc" -v b="$d_total" 'BEGIN{printf "%.2f", (a*100.0)/b}')"
            fi
        fi

        prev_total="$cpu_total"
        prev_proc="$proc_total"

        local rss_mb
        rss_mb="$(awk -v p="$rss_pages" -v s="$pagesize" 'BEGIN{printf "%.2f", (p*s)/(1024*1024)}')"

        local temp_c="-1"
        if [[ -f /sys/class/thermal/thermal_zone0/temp ]]; then
            temp_c="$(awk '{printf "%.1f", $1/1000.0}' /sys/class/thermal/thermal_zone0/temp 2>/dev/null || echo -1)"
        fi

        local rknpu="NA"
        if [[ -f /sys/kernel/debug/rknpu/load ]]; then
            rknpu="$(cat /sys/kernel/debug/rknpu/load 2>/dev/null | tr -d '\n' | tr ',' ';' || echo NA)"
        fi

        echo "$sec,$cpu_pct,$rss_mb,$temp_c,$rknpu" >> "$csv"

        sleep 1
        ((sec++)) || true
    done
}

summarize_round() {
    local log_file="$1"
    local csv_file="$2"
    local summary_file="$3"

    local final_line
    final_line="$(grep -E 'mpp_encoder_demo done:' "$log_file" | tail -n 1 || true)"

    local input_frames=0
    local output_packets=0
    local encode_frames=0
    local run_seconds=0

    if [[ -n "$final_line" ]]; then
        input_frames="$(echo "$final_line" | sed -n 's/.*input_frames=\([0-9]\+\).*/\1/p')"
        output_packets="$(echo "$final_line" | sed -n 's/.*output_packets=\([0-9]\+\).*/\1/p')"
        encode_frames="$(echo "$final_line" | sed -n 's/.*encode_frames=\([0-9]\+\).*/\1/p')"
        run_seconds="$(echo "$final_line" | sed -n 's/.*run_seconds=\([-0-9]\+\).*/\1/p')"
    fi

    local avg_fps="0.00"
    if [[ "$run_seconds" -gt 0 ]]; then
        avg_fps="$(awk -v f="$encode_frames" -v t="$run_seconds" 'BEGIN{printf "%.2f", f/t}')"
    fi

    local cpu_avg rss_peak temp_peak
    cpu_avg="$(awk -F, 'NR>1 {s+=$2;n++} END{if(n>0) printf "%.2f", s/n; else print "0.00"}' "$csv_file")"
    rss_peak="$(awk -F, 'NR>1 && $3>m {m=$3} END{if(m=="") m=0; printf "%.2f", m}' "$csv_file")"
    temp_peak="$(awk -F, 'NR>1 && $4>m {m=$4} END{if(m=="") m=-1; printf "%.1f", m}' "$csv_file")"

    {
        echo "final_line=$final_line"
        echo "input_frames=$input_frames"
        echo "output_packets=$output_packets"
        echo "encode_frames=$encode_frames"
        echo "run_seconds=$run_seconds"
        echo "avg_fps=$avg_fps"
        echo "cpu_avg_pct=$cpu_avg"
        echo "rss_peak_mb=$rss_peak"
        echo "temp_peak_c=$temp_peak"
    } > "$summary_file"
}

echo "[suite] output dir: $RUN_DIR"
echo "[suite] rounds=$ROUNDS duration_s=$DURATION_S"
echo "[suite] cmd: ${CMD[*]}"

for ((i=1; i<=ROUNDS; ++i)); do
    ROUND_DIR="$RUN_DIR/round_$i"
    mkdir -p "$ROUND_DIR"

    LOG_FILE="$ROUND_DIR/mpp.log"
    CSV_FILE="$ROUND_DIR/runtime.csv"
    SUM_FILE="$ROUND_DIR/summary.txt"

    echo "[suite] round $i/$ROUNDS start"

    set +e
    "${CMD[@]}" > "$LOG_FILE" 2>&1 &
    PID=$!
    set -e

    sample_runtime_stats "$PID" "$CSV_FILE"

    wait "$PID" || true

    summarize_round "$LOG_FILE" "$CSV_FILE" "$SUM_FILE"

    echo "[suite] round $i summary:"
    cat "$SUM_FILE"
    echo

done

GLOBAL_SUM="$RUN_DIR/overall_summary.csv"
echo "round,avg_fps,cpu_avg_pct,rss_peak_mb,temp_peak_c" > "$GLOBAL_SUM"
for ((i=1; i<=ROUNDS; ++i)); do
    SUM_FILE="$RUN_DIR/round_$i/summary.txt"
    avg_fps="$(grep '^avg_fps=' "$SUM_FILE" | cut -d= -f2)"
    cpu_avg="$(grep '^cpu_avg_pct=' "$SUM_FILE" | cut -d= -f2)"
    rss_peak="$(grep '^rss_peak_mb=' "$SUM_FILE" | cut -d= -f2)"
    temp_peak="$(grep '^temp_peak_c=' "$SUM_FILE" | cut -d= -f2)"
    echo "$i,$avg_fps,$cpu_avg,$rss_peak,$temp_peak" >> "$GLOBAL_SUM"
done

echo "[suite] done. overall summary: $GLOBAL_SUM"
