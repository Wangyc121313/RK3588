#!/usr/bin/env python3
"""Aggregate PhaseC metrics from telemetry and pseudo-label outputs.

Inputs:
- run_dir produced by scripts/run_phasec_suite.sh, containing scene subfolders.

Outputs:
- stdout summary
- optional markdown/csv table
"""

import argparse
import csv
import glob
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Tuple


@dataclass
class TrackObj:
    track_id: int
    class_id: int
    confirmed: bool
    ghost: bool
    left: float
    top: float
    right: float
    bottom: float
    distance_m: float


def percentile(values: List[float], p: float) -> float:
    if not values:
        return float("nan")
    if p <= 0:
        return min(values)
    if p >= 100:
        return max(values)
    vals = sorted(values)
    rank = (len(vals) - 1) * (p / 100.0)
    lo = int(math.floor(rank))
    hi = int(math.ceil(rank))
    if lo == hi:
        return vals[lo]
    frac = rank - lo
    return vals[lo] * (1.0 - frac) + vals[hi] * frac


def mean(values: List[float]) -> float:
    if not values:
        return float("nan")
    return sum(values) / float(len(values))


def stddev(values: List[float]) -> float:
    if len(values) < 2:
        return float("nan")
    mu = mean(values)
    return math.sqrt(sum((x - mu) ** 2 for x in values) / float(len(values)))


def iou(a: TrackObj, b: TrackObj) -> float:
    inter_left = max(a.left, b.left)
    inter_top = max(a.top, b.top)
    inter_right = min(a.right, b.right)
    inter_bottom = min(a.bottom, b.bottom)
    inter_w = max(0.0, inter_right - inter_left)
    inter_h = max(0.0, inter_bottom - inter_top)
    inter_area = inter_w * inter_h

    area_a = max(0.0, a.right - a.left) * max(0.0, a.bottom - a.top)
    area_b = max(0.0, b.right - b.left) * max(0.0, b.bottom - b.top)
    denom = area_a + area_b - inter_area
    if denom <= 1e-6:
        return 0.0
    return inter_area / denom


def load_pseudo(path_pattern: str) -> Tuple[Dict[int, List[TrackObj]], int]:
    frames: Dict[int, List[TrackObj]] = {}
    json_errors = 0
    for path in sorted(glob.glob(path_pattern)):
        p = Path(path)
        if not p.is_file():
            continue
        with p.open("r", encoding="utf-8") as f:
            for raw in f:
                text = raw.strip()
                if not text:
                    continue
                try:
                    row = json.loads(text)
                except json.JSONDecodeError:
                    json_errors += 1
                    continue

                frame_id = int(row.get("frame_id", -1))
                if frame_id < 0:
                    continue

                objs: List[TrackObj] = []
                for obj in row.get("objects", []):
                    track_id = int(obj.get("track_id", -1))
                    if track_id < 0:
                        continue
                    bbox = obj.get("bbox", {})
                    if not isinstance(bbox, dict):
                        continue
                    objs.append(
                        TrackObj(
                            track_id=track_id,
                            class_id=int(obj.get("class_id", -1)),
                            confirmed=bool(obj.get("track_confirmed", False)),
                            ghost=bool(obj.get("track_is_ghost", False)),
                            left=float(bbox.get("left", 0.0)),
                            top=float(bbox.get("top", 0.0)),
                            right=float(bbox.get("right", 0.0)),
                            bottom=float(bbox.get("bottom", 0.0)),
                            distance_m=float(obj.get("distance_m", -1.0)),
                        )
                    )
                frames[frame_id] = objs
    return frames, json_errors


def load_latency(telemetry_path: Path) -> List[float]:
    latencies: List[float] = []
    if not telemetry_path.is_file():
        return latencies
    with telemetry_path.open("r", encoding="utf-8") as f:
        for raw in f:
            text = raw.strip()
            if not text:
                continue
            try:
                row = json.loads(text)
            except json.JSONDecodeError:
                continue
            value = row.get("capture_to_encode_ms", None)
            if isinstance(value, (int, float)) and value >= 0:
                latencies.append(float(value))
    return latencies


def continuity_metrics(frames: Dict[int, List[TrackObj]], iou_threshold: float = 0.30) -> Dict[str, float]:
    frame_ids = sorted(frames.keys())
    retention_total = 0
    retention_kept = 0
    switch_pairs = 0
    switch_events = 0

    track_frames: Dict[int, List[int]] = {}
    for frame_id in frame_ids:
        for obj in frames[frame_id]:
            if obj.confirmed:
                track_frames.setdefault(obj.track_id, []).append(frame_id)

    for idx in range(len(frame_ids) - 1):
        f0 = frame_ids[idx]
        f1 = frame_ids[idx + 1]
        objs0 = [o for o in frames[f0] if o.confirmed]
        objs1 = [o for o in frames[f1] if o.confirmed]

        ids0 = {o.track_id for o in objs0}
        ids1 = {o.track_id for o in objs1}
        retention_total += len(ids0)
        retention_kept += len(ids0 & ids1)

        for o0 in objs0:
            candidates = [o1 for o1 in objs1 if o1.class_id == o0.class_id]
            if not candidates:
                continue
            best = None
            best_iou = -1.0
            for o1 in candidates:
                v = iou(o0, o1)
                if v > best_iou:
                    best_iou = v
                    best = o1
            if best is None or best_iou < iou_threshold:
                continue
            switch_pairs += 1
            if best.track_id != o0.track_id:
                switch_events += 1

    fragmented_tracks = 0
    fragmentation_events = 0
    for _, frame_list in track_frames.items():
        u = sorted(set(frame_list))
        if len(u) <= 1:
            continue
        segments = 1
        prev = u[0]
        for cur in u[1:]:
            if cur - prev > 1:
                segments += 1
            prev = cur
        if segments > 1:
            fragmented_tracks += 1
            fragmentation_events += (segments - 1)

    track_count = max(1, len(track_frames))
    return {
        "track_retention_ratio": float(retention_kept) / float(max(1, retention_total)),
        "id_switch_proxy_rate": float(switch_events) / float(max(1, switch_pairs)),
        "track_fragmentation_rate": float(fragmentation_events) / float(track_count),
        "track_count": float(track_count),
        "retention_samples": float(retention_total),
        "switch_samples": float(switch_pairs),
        "fragmented_tracks": float(fragmented_tracks),
    }


def distance_jitter_metric(frames: Dict[int, List[TrackObj]]) -> Tuple[float, float, int, int]:
    per_track_dist: Dict[int, List[float]] = {}
    valid_samples = 0
    for _, objs in frames.items():
        for obj in objs:
            if not obj.confirmed or obj.ghost:
                continue
            if obj.distance_m < 0:
                continue
            valid_samples += 1
            per_track_dist.setdefault(obj.track_id, []).append(obj.distance_m)

    if not per_track_dist:
        return float("nan"), float("nan"), 0, valid_samples

    best_track = max(per_track_dist.keys(), key=lambda k: len(per_track_dist[k]))
    series = per_track_dist[best_track]
    jitter_std = stddev(series)
    jitter_span = percentile(series, 95) - percentile(series, 5)
    return jitter_std, jitter_span, best_track, valid_samples


def scene_metrics(scene_dir: Path) -> Dict[str, object]:
    pseudo_pattern = str(scene_dir / "pseudo_labels.jsonl*")
    telemetry_path = scene_dir / "telemetry.jsonl"
    frames, pseudo_errors = load_pseudo(pseudo_pattern)

    cont = continuity_metrics(frames)
    jitter_std, jitter_span, best_track, valid_dist = distance_jitter_metric(frames)

    latencies = load_latency(telemetry_path)
    latency_mean = mean(latencies)
    latency_p95 = percentile(latencies, 95)

    return {
        "scene": scene_dir.name,
        "frame_count": len(frames),
        "pseudo_json_errors": pseudo_errors,
        "distance_jitter_std_m": jitter_std,
        "distance_jitter_p95_p5_m": jitter_span,
        "distance_best_track_id": best_track,
        "distance_valid_samples": valid_dist,
        "track_retention_ratio": cont["track_retention_ratio"],
        "id_switch_proxy_rate": cont["id_switch_proxy_rate"],
        "track_fragmentation_rate": cont["track_fragmentation_rate"],
        "track_count": int(cont["track_count"]),
        "retention_samples": int(cont["retention_samples"]),
        "switch_samples": int(cont["switch_samples"]),
        "latency_mean_ms": latency_mean,
        "latency_p95_ms": latency_p95,
    }


def to_text(v: object, digits: int = 3) -> str:
    if isinstance(v, float):
        if math.isnan(v):
            return "nan"
        return f"{v:.{digits}f}"
    return str(v)


def write_markdown(path: Path, rows: List[Dict[str, object]]) -> None:
    header = [
        "scene",
        "frame_count",
        "distance_jitter_std_m",
        "distance_jitter_p95_p5_m",
        "track_retention_ratio",
        "id_switch_proxy_rate",
        "track_fragmentation_rate",
        "latency_mean_ms",
        "latency_p95_ms",
    ]
    lines = []
    lines.append("# PhaseC 指标表")
    lines.append("")
    lines.append("| " + " | ".join(header) + " |")
    lines.append("|" + "|".join(["---"] * len(header)) + "|")
    for row in rows:
        vals = [to_text(row.get(k, "")) for k in header]
        lines.append("| " + " | ".join(vals) + " |")
    lines.append("")
    lines.append("说明：distance_jitter 基于已确认且非 ghost 的主轨迹距离序列计算。")
    path.write_text("\n".join(lines), encoding="utf-8")


def write_csv(path: Path, rows: List[Dict[str, object]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fields = [
        "scene",
        "frame_count",
        "pseudo_json_errors",
        "distance_jitter_std_m",
        "distance_jitter_p95_p5_m",
        "distance_best_track_id",
        "distance_valid_samples",
        "track_retention_ratio",
        "id_switch_proxy_rate",
        "track_fragmentation_rate",
        "track_count",
        "retention_samples",
        "switch_samples",
        "latency_mean_ms",
        "latency_p95_ms",
    ]
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Aggregate PhaseC metrics from telemetry and pseudo labels")
    parser.add_argument("--run-dir", required=True, help="PhaseC run directory")
    parser.add_argument("--markdown-out", default="", help="Optional markdown output path")
    parser.add_argument("--csv-out", default="", help="Optional csv output path")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    run_dir = Path(args.run_dir)
    if not run_dir.is_dir():
        print(f"error: run_dir not found: {run_dir}")
        return 2

    scene_dirs = [
        p for p in sorted(run_dir.iterdir())
        if p.is_dir() and (p / "telemetry.jsonl").exists()
    ]
    if not scene_dirs:
        print("error: no scene directories with telemetry.jsonl found")
        return 2

    rows: List[Dict[str, object]] = [scene_metrics(scene) for scene in scene_dirs]

    print("[phasec_metrics]")
    for row in rows:
        print(
            f"scene={row['scene']} frames={row['frame_count']} "
            f"jitter_std_m={to_text(row['distance_jitter_std_m'])} "
            f"retention={to_text(row['track_retention_ratio'])} "
            f"id_switch={to_text(row['id_switch_proxy_rate'])} "
            f"lat_mean_ms={to_text(row['latency_mean_ms'])} lat_p95_ms={to_text(row['latency_p95_ms'])}"
        )

    md_path = Path(args.markdown_out) if args.markdown_out else (run_dir / "metrics_table.md")
    csv_path = Path(args.csv_out) if args.csv_out else (run_dir / "metrics_table.csv")
    write_markdown(md_path, rows)
    write_csv(csv_path, rows)
    print(f"markdown={md_path}")
    print(f"csv={csv_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
