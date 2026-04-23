#!/usr/bin/env bash
set -euo pipefail

# Run 3 standard scenes and generate resume-friendly summary.
#
# Usage:
#   scripts/run_resume_three_scenarios.sh [camera_device] [width] [height] [run_seconds] [out_root]
# Example:
#   scripts/run_resume_three_scenarios.sh /dev/video0 640 480 75 reports/phasec

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

CAMERA_DEVICE="${1:-/dev/video0}"
CAMERA_WIDTH="${2:-640}"
CAMERA_HEIGHT="${3:-480}"
RUN_SECONDS="${4:-75}"
OUT_ROOT="${5:-reports/phasec}"

export PHASEC_NO_PAUSE="${PHASEC_NO_PAUSE:-1}"

echo "[resume_3scene] start"
echo "[resume_3scene] camera=$CAMERA_DEVICE ${CAMERA_WIDTH}x${CAMERA_HEIGHT} run_seconds=$RUN_SECONDS"
echo "[resume_3scene] out_root=$OUT_ROOT"

scripts/run_phasec_suite.sh \
  "$CAMERA_DEVICE" \
  "$CAMERA_WIDTH" \
  "$CAMERA_HEIGHT" \
  "$RUN_SECONDS" \
  "$OUT_ROOT"

RUN_DIR="$(readlink -f "$OUT_ROOT/latest")"
if [[ -z "$RUN_DIR" || ! -d "$RUN_DIR" ]]; then
  echo "error: cannot resolve latest run dir from $OUT_ROOT/latest"
  exit 2
fi

METRICS_CSV="$RUN_DIR/metrics_table.csv"
if [[ ! -f "$METRICS_CSV" ]]; then
  echo "error: metrics csv not found: $METRICS_CSV"
  exit 2
fi

GATE_TXT="$RUN_DIR/gate_result.txt"
if python3 tools/diagnostics/check_phasec_gate.py --run-dir "$RUN_DIR" > "$GATE_TXT" 2>&1; then
  GATE_STATUS="PASS"
else
  GATE_STATUS="FAIL"
fi

SUMMARY_MD="$RUN_DIR/resume_metrics_3scene.md"
SUMMARY_JSON="$RUN_DIR/resume_metrics_3scene.json"

python3 - "$METRICS_CSV" "$GATE_STATUS" "$SUMMARY_MD" "$SUMMARY_JSON" <<'PY'
import csv
import json
import math
import statistics
import sys

csv_path, gate_status, md_path, json_path = sys.argv[1:5]

rows = []
with open(csv_path, "r", encoding="utf-8", newline="") as f:
    reader = csv.DictReader(f)
    rows = list(reader)

if not rows:
    raise SystemExit("no rows in metrics csv")

def f(row, key):
    try:
        return float(row.get(key, "nan"))
    except ValueError:
        return float("nan")

def clean(values):
    return [v for v in values if not math.isnan(v)]

scene_stats = []
for r in rows:
    scene_stats.append(
        {
            "scene": r.get("scene", "unknown"),
            "frame_count": int(float(r.get("frame_count", "0") or 0)),
            "distance_jitter_std_m": f(r, "distance_jitter_std_m"),
            "distance_jitter_p95_p5_m": f(r, "distance_jitter_p95_p5_m"),
            "track_retention_ratio": f(r, "track_retention_ratio"),
            "id_switch_proxy_rate": f(r, "id_switch_proxy_rate"),
            "track_fragmentation_rate": f(r, "track_fragmentation_rate"),
            "latency_mean_ms": f(r, "latency_mean_ms"),
            "latency_p95_ms": f(r, "latency_p95_ms"),
        }
    )

ret = clean([x["track_retention_ratio"] for x in scene_stats])
id_sw = clean([x["id_switch_proxy_rate"] for x in scene_stats])
frag = clean([x["track_fragmentation_rate"] for x in scene_stats])
lat95 = clean([x["latency_p95_ms"] for x in scene_stats])
jit = clean([x["distance_jitter_std_m"] for x in scene_stats])
frames = [x["frame_count"] for x in scene_stats]

summary = {
    "gate_status": gate_status,
    "scene_count": len(scene_stats),
    "total_frames": sum(frames),
    "avg_track_retention_ratio": statistics.fmean(ret) if ret else float("nan"),
    "worst_track_retention_ratio": min(ret) if ret else float("nan"),
    "avg_id_switch_proxy_rate": statistics.fmean(id_sw) if id_sw else float("nan"),
    "worst_id_switch_proxy_rate": max(id_sw) if id_sw else float("nan"),
    "avg_track_fragmentation_rate": statistics.fmean(frag) if frag else float("nan"),
    "worst_track_fragmentation_rate": max(frag) if frag else float("nan"),
    "avg_latency_p95_ms": statistics.fmean(lat95) if lat95 else float("nan"),
    "worst_latency_p95_ms": max(lat95) if lat95 else float("nan"),
    "avg_distance_jitter_std_m": statistics.fmean(jit) if jit else float("nan"),
    "worst_distance_jitter_std_m": max(jit) if jit else float("nan"),
    "scenes": scene_stats,
}

with open(json_path, "w", encoding="utf-8") as f:
    json.dump(summary, f, ensure_ascii=False, indent=2)

def fmt(x, d=3):
    if isinstance(x, float) and math.isnan(x):
        return "nan"
    if isinstance(x, float):
        return f"{x:.{d}f}"
    return str(x)

with open(md_path, "w", encoding="utf-8") as f:
    f.write("# 三场景简历指标摘要\n\n")
    f.write(f"- 验收结果: **{gate_status}**\n")
    f.write(f"- 场景数量: {summary['scene_count']}\n")
    f.write(f"- 总帧数: {summary['total_frames']}\n\n")

    f.write("## 核心指标（跨场景汇总）\n\n")
    f.write(f"- Track 保持率均值: {fmt(summary['avg_track_retention_ratio'])}\n")
    f.write(f"- Track 保持率最差: {fmt(summary['worst_track_retention_ratio'])}\n")
    f.write(f"- ID 切换代理率均值: {fmt(summary['avg_id_switch_proxy_rate'])}\n")
    f.write(f"- ID 切换代理率最差: {fmt(summary['worst_id_switch_proxy_rate'])}\n")
    f.write(f"- 轨迹碎片率均值: {fmt(summary['avg_track_fragmentation_rate'])}\n")
    f.write(f"- 轨迹碎片率最差: {fmt(summary['worst_track_fragmentation_rate'])}\n")
    f.write(f"- 端到端时延P95均值(ms): {fmt(summary['avg_latency_p95_ms'])}\n")
    f.write(f"- 端到端时延P95最差(ms): {fmt(summary['worst_latency_p95_ms'])}\n")
    f.write(f"- 距离抖动std均值(m): {fmt(summary['avg_distance_jitter_std_m'])}\n")
    f.write(f"- 距离抖动std最差(m): {fmt(summary['worst_distance_jitter_std_m'])}\n\n")

    f.write("## 分场景指标\n\n")
    f.write("| scene | frame_count | retention | id_switch | fragmentation | latency_p95_ms | jitter_std_m |\n")
    f.write("|---|---:|---:|---:|---:|---:|---:|\n")
    for s in scene_stats:
        f.write(
            f"| {s['scene']} | {s['frame_count']} | {fmt(s['track_retention_ratio'])} | "
            f"{fmt(s['id_switch_proxy_rate'])} | {fmt(s['track_fragmentation_rate'])} | "
            f"{fmt(s['latency_p95_ms'])} | {fmt(s['distance_jitter_std_m'])} |\n"
        )

print(md_path)
print(json_path)
PY


echo "[resume_3scene] run_dir=$RUN_DIR"
echo "[resume_3scene] gate_status=$GATE_STATUS"
echo "[resume_3scene] gate_log=$GATE_TXT"
echo "[resume_3scene] summary_md=$SUMMARY_MD"
echo "[resume_3scene] summary_json=$SUMMARY_JSON"

if [[ "$GATE_STATUS" != "PASS" ]]; then
  echo "[resume_3scene] gate check failed"
  exit 1
fi

echo "[resume_3scene] done"
