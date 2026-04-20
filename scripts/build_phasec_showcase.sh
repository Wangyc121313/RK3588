#!/usr/bin/env bash
set -euo pipefail

RUN_DIR="${1:-}"
if [[ -z "$RUN_DIR" || ! -d "$RUN_DIR" ]]; then
  echo "usage: scripts/build_phasec_showcase.sh <phasec_run_dir>"
  exit 2
fi

SHOWCASE_DIR="$RUN_DIR/showcase"
mkdir -p "$SHOWCASE_DIR"

if [[ -f "$RUN_DIR/metrics_table.md" ]]; then
  cp "$RUN_DIR/metrics_table.md" "$SHOWCASE_DIR/"
fi
if [[ -f "$RUN_DIR/metrics_table.csv" ]]; then
  cp "$RUN_DIR/metrics_table.csv" "$SHOWCASE_DIR/"
fi

CONFIG_MD="$SHOWCASE_DIR/config_summary.md"
{
  echo "# PhaseC 配置说明"
  echo
  for scene in static_target approaching_target crossing_occlusion; do
    SCENE_DIR="$RUN_DIR/$scene"
    if [[ ! -d "$SCENE_DIR" ]]; then
      continue
    fi
    echo "## ${scene}"
    if [[ -f "$SCENE_DIR/config.env" ]]; then
      echo '```bash'
      cat "$SCENE_DIR/config.env"
      echo '```'
    else
      echo "(无 config.env)"
    fi
    echo
  done
} > "$CONFIG_MD"

DEMO_MD="$SHOWCASE_DIR/demo_assets.md"
{
  echo "# PhaseC 演示素材"
  echo
  echo "下列文件来自三类标准场景采集："
  echo
  for scene in static_target approaching_target crossing_occlusion; do
    SCENE_DIR="$RUN_DIR/$scene"
    if [[ ! -d "$SCENE_DIR" ]]; then
      continue
    fi
    echo "## ${scene}"
    for f in telemetry.jsonl pseudo_labels.jsonl tracking_metrics.txt video.h264; do
      if [[ -f "$SCENE_DIR/$f" ]]; then
        echo "- $scene/$f"
      fi
    done
    echo
  done
  echo "播放 H264 示例："
  echo '```bash'
  echo "ffplay -fflags nobuffer -flags low_delay $RUN_DIR/static_target/video.h264"
  echo '```'
} > "$DEMO_MD"

for scene in static_target approaching_target crossing_occlusion; do
  SCENE_DIR="$RUN_DIR/$scene"
  [[ -d "$SCENE_DIR" ]] || continue
  mkdir -p "$SHOWCASE_DIR/$scene"
  for f in telemetry.jsonl pseudo_labels.jsonl tracking_metrics.txt video.h264; do
    if [[ -f "$SCENE_DIR/$f" ]]; then
      cp "$SCENE_DIR/$f" "$SHOWCASE_DIR/$scene/"
    fi
  done

done

echo "showcase_dir=$SHOWCASE_DIR"
