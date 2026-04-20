#!/usr/bin/env python3
"""Analyze tracking quality from RK3588 pseudo-label JSONL files.

Metrics (proxy, no external ground truth):
- track_retention_ratio: ratio of track IDs in frame t that still exist in frame t+1
- id_switch_proxy_rate: IoU-based adjacent-frame matching where class stays same but track_id changes
- track_fragmentation_rate: extra disjoint segments per track sequence

Usage:
  python3 tools/diagnostics/analyze_tracking_metrics.py /tmp/rk3588_pseudo_labels.jsonl
  python3 tools/diagnostics/analyze_tracking_metrics.py --glob '/tmp/rk3588_pseudo_labels.jsonl*'
"""

import argparse
import glob
import json
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


def iter_paths(path: str, pattern: str) -> Iterable[Path]:
    if path:
        p = Path(path)
        if p.exists() and p.is_file():
            yield p
        return

    for matched in sorted(glob.glob(pattern)):
        p = Path(matched)
        if p.is_file():
            yield p


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


def load_frames(paths: List[Path]) -> Tuple[Dict[str, Dict[int, List[TrackObj]]], int, int]:
    by_seq: Dict[str, Dict[int, List[TrackObj]]] = {}
    frame_rows = 0
    json_errors = 0

    for path in paths:
        with path.open("r", encoding="utf-8") as f:
            for raw in f:
                text = raw.strip()
                if not text:
                    continue
                try:
                    row = json.loads(text)
                except json.JSONDecodeError:
                    json_errors += 1
                    continue

                frame_rows += 1
                seq = str(row.get("sequence_id", "default_seq"))
                frame_id = int(row.get("frame_id", -1))
                if frame_id < 0:
                    continue

                objects = row.get("objects", [])
                if not isinstance(objects, list):
                    continue

                frame_objs: List[TrackObj] = []
                for obj in objects:
                    track_id = int(obj.get("track_id", -1))
                    if track_id < 0:
                        continue
                    bbox = obj.get("bbox", {})
                    if not isinstance(bbox, dict):
                        continue

                    frame_objs.append(
                        TrackObj(
                            track_id=track_id,
                            class_id=int(obj.get("class_id", -1)),
                            confirmed=bool(obj.get("track_confirmed", False)),
                            ghost=bool(obj.get("track_is_ghost", False)),
                            left=float(bbox.get("left", 0.0)),
                            top=float(bbox.get("top", 0.0)),
                            right=float(bbox.get("right", 0.0)),
                            bottom=float(bbox.get("bottom", 0.0)),
                        )
                    )

                by_seq.setdefault(seq, {})[frame_id] = frame_objs

    return by_seq, frame_rows, json_errors


def compute_metrics(by_seq: Dict[str, Dict[int, List[TrackObj]]], iou_th: float) -> Dict[str, float]:
    retention_total = 0
    retention_kept = 0

    switch_pairs = 0
    switch_events = 0

    track_frames: Dict[Tuple[str, int], List[int]] = {}
    confirmed_objects = 0
    ghost_objects = 0

    for seq, frame_map in by_seq.items():
        frame_ids = sorted(frame_map.keys())
        for frame_id in frame_ids:
            for obj in frame_map[frame_id]:
                if obj.confirmed:
                    confirmed_objects += 1
                if obj.ghost:
                    ghost_objects += 1
                track_frames.setdefault((seq, obj.track_id), []).append(frame_id)

        for idx in range(len(frame_ids) - 1):
            f0 = frame_ids[idx]
            f1 = frame_ids[idx + 1]
            objs0 = [o for o in frame_map[f0] if o.confirmed]
            objs1 = [o for o in frame_map[f1] if o.confirmed]

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
                if best is None or best_iou < iou_th:
                    continue
                switch_pairs += 1
                if best.track_id != o0.track_id:
                    switch_events += 1

    fragmented_tracks = 0
    fragmentation_events = 0
    total_track_span = 0

    for _, frames in track_frames.items():
        unique_sorted = sorted(set(frames))
        if not unique_sorted:
            continue
        total_track_span += len(unique_sorted)

        segments = 1
        prev = unique_sorted[0]
        for cur in unique_sorted[1:]:
            if cur - prev > 1:
                segments += 1
            prev = cur

        if segments > 1:
            fragmented_tracks += 1
            fragmentation_events += segments - 1

    track_count = max(1, len(track_frames))
    metrics = {
        "sequence_count": float(len(by_seq)),
        "track_count": float(len(track_frames)),
        "confirmed_object_count": float(confirmed_objects),
        "ghost_object_count": float(ghost_objects),
        "track_retention_ratio": float(retention_kept) / float(max(1, retention_total)),
        "id_switch_proxy_rate": float(switch_events) / float(max(1, switch_pairs)),
        "track_fragmentation_rate": float(fragmentation_events) / float(track_count),
        "fragmented_track_ratio": float(fragmented_tracks) / float(track_count),
        "avg_track_observed_frames": float(total_track_span) / float(track_count),
        "retention_samples": float(retention_total),
        "switch_samples": float(switch_pairs),
    }
    return metrics


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze tracking quality from pseudo labels")
    parser.add_argument("path", nargs="?", default="", help="Single JSONL file path")
    parser.add_argument("--glob", default="", help="Glob pattern, e.g. '/tmp/rk3588_pseudo_labels.jsonl*'")
    parser.add_argument("--iou-threshold", type=float, default=0.30, help="IoU threshold for ID-switch proxy")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.path and not args.glob:
        print("error: provide a file path or --glob pattern")
        return 2

    paths = list(iter_paths(args.path, args.glob if args.glob else ""))
    if not paths:
        print("error: no input files found")
        return 2

    by_seq, frame_rows, json_errors = load_frames(paths)
    metrics = compute_metrics(by_seq, max(0.0, min(1.0, args.iou_threshold)))

    print("[tracking_metrics]")
    print(f"files={len(paths)} frame_rows={frame_rows} json_errors={json_errors}")
    print(f"sequences={int(metrics['sequence_count'])} tracks={int(metrics['track_count'])}")
    print(
        "core: "
        f"retention={metrics['track_retention_ratio']:.3f} "
        f"id_switch_proxy={metrics['id_switch_proxy_rate']:.3f} "
        f"fragmentation={metrics['track_fragmentation_rate']:.3f}"
    )
    print(
        "aux: "
        f"fragmented_ratio={metrics['fragmented_track_ratio']:.3f} "
        f"avg_track_frames={metrics['avg_track_observed_frames']:.2f} "
        f"ghost_objects={int(metrics['ghost_object_count'])}"
    )
    print(
        "samples: "
        f"retention={int(metrics['retention_samples'])} "
        f"switch={int(metrics['switch_samples'])}"
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
