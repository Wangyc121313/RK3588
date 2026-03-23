#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   scripts/perf_stress_suite.sh [duration_s] [rounds] [output_dir] [-- extra mpp args...]
# Example:
#   scripts/perf_stress_suite.sh 120 3 perf_runs

DURATION_S="${1:-120}"
ROUNDS="${2:-3}"
OUT_DIR="${3:-reports/perf}"
shift $(( $# >= 3 ? 3 : $# )) || true

TARGET_FPS="${TARGET_FPS:-25}"
NET_IFACE="${NET_IFACE:-}"

EXTRA_ARGS=()
if [[ "${1:-}" == "--" ]]; then
    shift
    EXTRA_ARGS=("$@")
fi

mkdir -p "$OUT_DIR"
TS="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$OUT_DIR/run_$TS"
mkdir -p "$RUN_DIR"
ln -sfn "$(basename "$RUN_DIR")" "$OUT_DIR/latest"

DEFAULT_CMD=(
    ./build/mpp_encoder_demo
    /dev/video1 640 480
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

detect_net_iface() {
    if [[ -n "$NET_IFACE" && -d "/sys/class/net/$NET_IFACE" ]]; then
        echo "$NET_IFACE"
        return
    fi
    local via
    via="$(ip route 2>/dev/null | awk '/default/ {print $5; exit}' || true)"
    if [[ -n "$via" && -d "/sys/class/net/$via" ]]; then
        echo "$via"
        return
    fi
    echo ""
}

read_proc_io_bytes() {
    local pid="$1"
    awk '/read_bytes/ {r=$2} /write_bytes/ {w=$2} END {print (r+0), (w+0)}' "/proc/$pid/io" 2>/dev/null || echo "0 0"
}

read_proc_ctx_switches() {
    local pid="$1"
    awk '/voluntary_ctxt_switches/ {v=$2} /nonvoluntary_ctxt_switches/ {n=$2} END {print (v+n+0)}' "/proc/$pid/status" 2>/dev/null || echo "0"
}

read_proc_fd_count() {
    local pid="$1"
    ls "/proc/$pid/fd" 2>/dev/null | wc -l | awk '{print $1+0}'
}

read_proc_threads() {
    local pid="$1"
    ls "/proc/$pid/task" 2>/dev/null | wc -l | awk '{print $1+0}'
}

read_net_bytes() {
    local iface="$1"
    if [[ -z "$iface" || ! -d "/sys/class/net/$iface" ]]; then
        echo "0 0"
        return
    fi
    local rx tx
    rx="$(cat "/sys/class/net/$iface/statistics/rx_bytes" 2>/dev/null || echo 0)"
    tx="$(cat "/sys/class/net/$iface/statistics/tx_bytes" 2>/dev/null || echo 0)"
    echo "$rx $tx"
}

extract_rknpu_load_pct() {
    if [[ ! -f /sys/kernel/debug/rknpu/load ]]; then
        echo "-1"
        return
    fi
    local raw
    raw="$(cat /sys/kernel/debug/rknpu/load 2>/dev/null || true)"
    if [[ -z "$raw" ]]; then
        echo "-1"
        return
    fi
    local pct
    pct="$(echo "$raw" | grep -Eo '[0-9]+' | head -n 1 || true)"
    echo "${pct:--1}"
}

percentile_from_csv_col() {
    local csv_file="$1"
    local col="$2"
    local pct="$3"
    local values_file="$4"
    awk -F, -v c="$col" 'NR>1 && $c!="" {print $c}' "$csv_file" | sort -n > "$values_file"
    local n
    n="$(wc -l < "$values_file" | awk '{print $1+0}')"
    if [[ "$n" -le 0 ]]; then
        echo "0"
        return
    fi
    local idx
    idx="$(awk -v n="$n" -v p="$pct" 'BEGIN{i=int((n-1)*p+1); if(i<1)i=1; if(i>n)i=n; print i}')"
    sed -n "${idx}p" "$values_file"
}

sample_runtime_stats() {
    local pid="$1"
    local csv="$2"
    local iface="$3"

    echo "sec,cpu_pct,rss_mb,vsz_mb,threads,fd_count,temp_c,io_read_kbps,io_write_kbps,ctx_switches_ps,net_rx_kbps,net_tx_kbps,rknpu_pct" > "$csv"

    local pagesize
    pagesize="$(getconf PAGESIZE)"

    local prev_total=0
    local prev_proc=0
    local prev_read=0
    local prev_write=0
    local prev_ctx=0
    local prev_rx=0
    local prev_tx=0
    local sec=0

    read -r prev_read prev_write <<< "$(read_proc_io_bytes "$pid")"
    prev_ctx="$(read_proc_ctx_switches "$pid")"
    read -r prev_rx prev_tx <<< "$(read_net_bytes "$iface")"

    while kill -0 "$pid" 2>/dev/null; do
        local proc_stat cpu_stat
        proc_stat="$(awk '{print $14, $15, $23, $24, $20}' /proc/$pid/stat 2>/dev/null || true)"
        cpu_stat="$(awk '/^cpu /{print $2+$3+$4+$5+$6+$7+$8+$9+$10}' /proc/stat 2>/dev/null || true)"

        if [[ -z "$proc_stat" || -z "$cpu_stat" ]]; then
            sleep 1
            ((sec++)) || true
            continue
        fi

        local utime stime vsz_bytes rss_pages threads_stat
        read -r utime stime vsz_bytes rss_pages threads_stat <<< "$proc_stat"
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
        local vsz_mb
        vsz_mb="$(awk -v b="$vsz_bytes" 'BEGIN{printf "%.2f", b/(1024*1024)}')"

        local fd_count threads
        fd_count="$(read_proc_fd_count "$pid")"
        threads="$(read_proc_threads "$pid")"
        if [[ "$threads" -le 0 ]]; then
            threads="$threads_stat"
        fi

        local temp_c="-1"
        if [[ -f /sys/class/thermal/thermal_zone0/temp ]]; then
            temp_c="$(awk '{printf "%.1f", $1/1000.0}' /sys/class/thermal/thermal_zone0/temp 2>/dev/null || echo -1)"
        fi

        local io_read io_write
        read -r io_read io_write <<< "$(read_proc_io_bytes "$pid")"
        local d_read=$((io_read - prev_read))
        local d_write=$((io_write - prev_write))
        prev_read="$io_read"
        prev_write="$io_write"
        if [[ "$d_read" -lt 0 ]]; then d_read=0; fi
        if [[ "$d_write" -lt 0 ]]; then d_write=0; fi
        local io_read_kbps io_write_kbps
        io_read_kbps="$(awk -v b="$d_read" 'BEGIN{printf "%.2f", b/1024.0}')"
        io_write_kbps="$(awk -v b="$d_write" 'BEGIN{printf "%.2f", b/1024.0}')"

        local ctx_now d_ctx
        ctx_now="$(read_proc_ctx_switches "$pid")"
        d_ctx=$((ctx_now - prev_ctx))
        prev_ctx="$ctx_now"
        if [[ "$d_ctx" -lt 0 ]]; then d_ctx=0; fi

        local rx_now tx_now
        read -r rx_now tx_now <<< "$(read_net_bytes "$iface")"
        local d_rx=$((rx_now - prev_rx))
        local d_tx=$((tx_now - prev_tx))
        prev_rx="$rx_now"
        prev_tx="$tx_now"
        if [[ "$d_rx" -lt 0 ]]; then d_rx=0; fi
        if [[ "$d_tx" -lt 0 ]]; then d_tx=0; fi
        local net_rx_kbps net_tx_kbps
        net_rx_kbps="$(awk -v b="$d_rx" 'BEGIN{printf "%.2f", b/1024.0}')"
        net_tx_kbps="$(awk -v b="$d_tx" 'BEGIN{printf "%.2f", b/1024.0}')"

        local rknpu_pct
        rknpu_pct="$(extract_rknpu_load_pct)"

        echo "$sec,$cpu_pct,$rss_mb,$vsz_mb,$threads,$fd_count,$temp_c,$io_read_kbps,$io_write_kbps,$d_ctx,$net_rx_kbps,$net_tx_kbps,$rknpu_pct" >> "$csv"

        sleep 1
        ((sec++)) || true
    done
}

summarize_round() {
    local log_file="$1"
    local csv_file="$2"
    local summary_file="$3"
    local exit_code="$4"

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

    local pkt_per_frame="0.000"
    local frame_delivery_ratio="0.000"
    if [[ "$input_frames" -gt 0 ]]; then
        pkt_per_frame="$(awk -v o="$output_packets" -v i="$input_frames" 'BEGIN{printf "%.3f", o/i}')"
        frame_delivery_ratio="$(awk -v o="$output_packets" -v i="$input_frames" 'BEGIN{r=o/i; if(r>1) r=1; if(r<0) r=0; printf "%.3f", r}')"
    fi

    local fps_util_pct="0.00"
    if [[ "$TARGET_FPS" -gt 0 ]]; then
        fps_util_pct="$(awk -v a="$avg_fps" -v t="$TARGET_FPS" 'BEGIN{printf "%.2f", (a*100.0)/t}')"
    fi

    local cpu_avg cpu_peak rss_avg rss_peak vsz_peak temp_avg temp_peak
    local io_r_avg io_r_peak io_w_avg io_w_peak
    local ctx_avg ctx_peak rx_avg rx_peak tx_avg tx_peak
    local threads_peak fd_peak rknpu_avg rknpu_peak

    cpu_avg="$(awk -F, 'NR>1 {s+=$2;n++} END{if(n>0) printf "%.2f", s/n; else print "0.00"}' "$csv_file")"
    cpu_peak="$(awk -F, 'NR>1 && $2>m {m=$2} END{if(m=="") m=0; printf "%.2f", m}' "$csv_file")"
    rss_avg="$(awk -F, 'NR>1 {s+=$3;n++} END{if(n>0) printf "%.2f", s/n; else print "0.00"}' "$csv_file")"
    rss_peak="$(awk -F, 'NR>1 && $3>m {m=$3} END{if(m=="") m=0; printf "%.2f", m}' "$csv_file")"
    vsz_peak="$(awk -F, 'NR>1 && $4>m {m=$4} END{if(m=="") m=0; printf "%.2f", m}' "$csv_file")"
    temp_avg="$(awk -F, 'NR>1 && $7>=0 {s+=$7;n++} END{if(n>0) printf "%.1f", s/n; else print "-1"}' "$csv_file")"
    temp_peak="$(awk -F, 'NR>1 && $7>m {m=$7} END{if(m=="") m=-1; printf "%.1f", m}' "$csv_file")"
    io_r_avg="$(awk -F, 'NR>1 {s+=$8;n++} END{if(n>0) printf "%.2f", s/n; else print "0.00"}' "$csv_file")"
    io_r_peak="$(awk -F, 'NR>1 && $8>m {m=$8} END{if(m=="") m=0; printf "%.2f", m}' "$csv_file")"
    io_w_avg="$(awk -F, 'NR>1 {s+=$9;n++} END{if(n>0) printf "%.2f", s/n; else print "0.00"}' "$csv_file")"
    io_w_peak="$(awk -F, 'NR>1 && $9>m {m=$9} END{if(m=="") m=0; printf "%.2f", m}' "$csv_file")"
    ctx_avg="$(awk -F, 'NR>1 {s+=$10;n++} END{if(n>0) printf "%.2f", s/n; else print "0.00"}' "$csv_file")"
    ctx_peak="$(awk -F, 'NR>1 && $10>m {m=$10} END{if(m=="") m=0; printf "%.0f", m}' "$csv_file")"
    rx_avg="$(awk -F, 'NR>1 {s+=$11;n++} END{if(n>0) printf "%.2f", s/n; else print "0.00"}' "$csv_file")"
    rx_peak="$(awk -F, 'NR>1 && $11>m {m=$11} END{if(m=="") m=0; printf "%.2f", m}' "$csv_file")"
    tx_avg="$(awk -F, 'NR>1 {s+=$12;n++} END{if(n>0) printf "%.2f", s/n; else print "0.00"}' "$csv_file")"
    tx_peak="$(awk -F, 'NR>1 && $12>m {m=$12} END{if(m=="") m=0; printf "%.2f", m}' "$csv_file")"
    threads_peak="$(awk -F, 'NR>1 && $5>m {m=$5} END{if(m=="") m=0; print int(m)}' "$csv_file")"
    fd_peak="$(awk -F, 'NR>1 && $6>m {m=$6} END{if(m=="") m=0; print int(m)}' "$csv_file")"
    rknpu_avg="$(awk -F, 'NR>1 && $13>=0 {s+=$13;n++} END{if(n>0) printf "%.2f", s/n; else print "-1"}' "$csv_file")"
    rknpu_peak="$(awk -F, 'NR>1 && $13>m {m=$13} END{if(m=="") m=-1; printf "%.0f", m}' "$csv_file")"

    local tmp_p95 tmp_p99 cpu_p95 cpu_p99
    tmp_p95="${csv_file}.cpu.p95.tmp"
    tmp_p99="${csv_file}.cpu.p99.tmp"
    cpu_p95="$(percentile_from_csv_col "$csv_file" 2 0.95 "$tmp_p95")"
    cpu_p99="$(percentile_from_csv_col "$csv_file" 2 0.99 "$tmp_p99")"
    rm -f "$tmp_p95" "$tmp_p99"

    {
        echo "exit_code=$exit_code"
        echo "final_line=$final_line"
        echo "input_frames=$input_frames"
        echo "output_packets=$output_packets"
        echo "encode_frames=$encode_frames"
        echo "run_seconds=$run_seconds"
        echo "avg_fps=$avg_fps"
        echo "target_fps=$TARGET_FPS"
        echo "fps_util_pct=$fps_util_pct"
        echo "packet_per_frame=$pkt_per_frame"
        echo "frame_delivery_ratio=$frame_delivery_ratio"
        echo "cpu_avg_pct=$cpu_avg"
        echo "cpu_p95_pct=$cpu_p95"
        echo "cpu_p99_pct=$cpu_p99"
        echo "cpu_peak_pct=$cpu_peak"
        echo "rss_avg_mb=$rss_avg"
        echo "rss_peak_mb=$rss_peak"
        echo "vsz_peak_mb=$vsz_peak"
        echo "threads_peak=$threads_peak"
        echo "fd_peak=$fd_peak"
        echo "temp_avg_c=$temp_avg"
        echo "temp_peak_c=$temp_peak"
        echo "io_read_avg_kbps=$io_r_avg"
        echo "io_read_peak_kbps=$io_r_peak"
        echo "io_write_avg_kbps=$io_w_avg"
        echo "io_write_peak_kbps=$io_w_peak"
        echo "ctx_switch_avg_ps=$ctx_avg"
        echo "ctx_switch_peak_ps=$ctx_peak"
        echo "net_rx_avg_kbps=$rx_avg"
        echo "net_rx_peak_kbps=$rx_peak"
        echo "net_tx_avg_kbps=$tx_avg"
        echo "net_tx_peak_kbps=$tx_peak"
        echo "rknpu_avg_pct=$rknpu_avg"
        echo "rknpu_peak_pct=$rknpu_peak"
    } > "$summary_file"
}

echo "[suite] output dir: $RUN_DIR"
echo "[suite] rounds=$ROUNDS duration_s=$DURATION_S"
IFACE="$(detect_net_iface)"
echo "[suite] net_if=${IFACE:-N/A} target_fps=$TARGET_FPS"
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

    sample_runtime_stats "$PID" "$CSV_FILE" "$IFACE"

    set +e
    wait "$PID"
    EXIT_CODE=$?
    set -e

    summarize_round "$LOG_FILE" "$CSV_FILE" "$SUM_FILE" "$EXIT_CODE"

    echo "[suite] round $i summary:"
    cat "$SUM_FILE"
    echo

done

GLOBAL_SUM="$RUN_DIR/overall_summary.csv"
echo "round,exit_code,avg_fps,fps_util_pct,frame_delivery_ratio,cpu_avg_pct,cpu_p95_pct,cpu_peak_pct,rss_peak_mb,threads_peak,fd_peak,temp_peak_c,rknpu_avg_pct,net_tx_avg_kbps" > "$GLOBAL_SUM"
for ((i=1; i<=ROUNDS; ++i)); do
    SUM_FILE="$RUN_DIR/round_$i/summary.txt"
    exit_code="$(grep '^exit_code=' "$SUM_FILE" | cut -d= -f2)"
    avg_fps="$(grep '^avg_fps=' "$SUM_FILE" | cut -d= -f2)"
    fps_util_pct="$(grep '^fps_util_pct=' "$SUM_FILE" | cut -d= -f2)"
    frame_ratio="$(grep '^frame_delivery_ratio=' "$SUM_FILE" | cut -d= -f2)"
    cpu_avg="$(grep '^cpu_avg_pct=' "$SUM_FILE" | cut -d= -f2)"
    cpu_p95="$(grep '^cpu_p95_pct=' "$SUM_FILE" | cut -d= -f2)"
    cpu_peak="$(grep '^cpu_peak_pct=' "$SUM_FILE" | cut -d= -f2)"
    rss_peak="$(grep '^rss_peak_mb=' "$SUM_FILE" | cut -d= -f2)"
    threads_peak="$(grep '^threads_peak=' "$SUM_FILE" | cut -d= -f2)"
    fd_peak="$(grep '^fd_peak=' "$SUM_FILE" | cut -d= -f2)"
    temp_peak="$(grep '^temp_peak_c=' "$SUM_FILE" | cut -d= -f2)"
    rknpu_avg="$(grep '^rknpu_avg_pct=' "$SUM_FILE" | cut -d= -f2)"
    net_tx_avg="$(grep '^net_tx_avg_kbps=' "$SUM_FILE" | cut -d= -f2)"
    echo "$i,$exit_code,$avg_fps,$fps_util_pct,$frame_ratio,$cpu_avg,$cpu_p95,$cpu_peak,$rss_peak,$threads_peak,$fd_peak,$temp_peak,$rknpu_avg,$net_tx_avg" >> "$GLOBAL_SUM"
done

REPORT_MD="$RUN_DIR/report.md"
{
    echo "# Performance Report"
    echo
    echo "- rounds: $ROUNDS"
    echo "- duration_s_each: $DURATION_S"
    echo "- target_fps: $TARGET_FPS"
    echo "- net_iface: ${IFACE:-N/A}"
    echo
    echo "## Overall Summary"
    echo
    echo '```csv'
    cat "$GLOBAL_SUM"
    echo '```'
    echo
    echo "## Artifacts"
    echo
    echo "- per-round logs: round_*/mpp.log"
    echo "- per-round samples: round_*/runtime.csv"
    echo "- per-round key-value summary: round_*/summary.txt"
    echo "- overall csv: overall_summary.csv"
} > "$REPORT_MD"

echo "[suite] done. overall summary: $GLOBAL_SUM"
echo "[suite] markdown report: $REPORT_MD"
