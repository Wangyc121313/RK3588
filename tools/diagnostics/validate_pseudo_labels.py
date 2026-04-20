#!/usr/bin/env python3
"""Validate RK3588 pseudo label JSONL outputs.

Usage:
  python3 tools/diagnostics/validate_pseudo_labels.py /tmp/rk3588_pseudo_labels.jsonl
  python3 tools/diagnostics/validate_pseudo_labels.py --glob '/tmp/rk3588_pseudo_labels.jsonl*'
"""

import argparse
import glob
import json
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

REQUIRED_FRAME_FIELDS = [
    "schema",
    "sequence_id",
    "source_fps",
    "camera_device",
    "frame_id",
    "timestamp_ms",
    "sensor_snapshot",
    "objects",
]

REQUIRED_SENSOR_SNAPSHOT_FIELDS = [
    "camera_fov_deg",
    "lidar_offset_deg",
    "lidar_fov_deg",
    "lidar_window_half_deg",
    "lidar_min_dist_m",
    "lidar_max_dist_m",
    "lidar_max_age_ms",
    "calibration_profile",
]

REQUIRED_OBJECT_FIELDS = [
    "class_id",
    "class_name",
    "confidence",
    "distance_m",
    "bbox",
    "track_id",
    "track_age_frames",
    "track_idle_frames",
    "track_confirmed",
    "track_is_ghost",
]

REQUIRED_BBOX_FIELDS = ["left", "top", "right", "bottom"]


def iter_paths(path: str, pattern: str) -> Iterable[Path]:
    if path:
        p = Path(path)
        if p.exists():
            yield p
        return

    for matched in sorted(glob.glob(pattern)):
        p = Path(matched)
        if p.is_file():
            yield p


def validate_file(path: Path) -> Dict[str, int]:
    stats = {
        "frames": 0,
        "objects": 0,
        "json_errors": 0,
        "missing_frame_fields": 0,
        "missing_sensor_snapshot_fields": 0,
        "missing_object_fields": 0,
        "missing_bbox_fields": 0,
        "track_confirmed": 0,
        "valid_distance": 0,
    }

    with path.open("r", encoding="utf-8") as f:
        for line_no, raw in enumerate(f, start=1):
            text = raw.strip()
            if not text:
                continue

            try:
                row = json.loads(text)
            except json.JSONDecodeError:
                stats["json_errors"] += 1
                continue

            stats["frames"] += 1
            missing_frame = [k for k in REQUIRED_FRAME_FIELDS if k not in row]
            if missing_frame:
                stats["missing_frame_fields"] += 1
                continue

            objects = row.get("objects", [])
            if not isinstance(objects, list):
                stats["missing_frame_fields"] += 1
                continue

            sensor_snapshot = row.get("sensor_snapshot")
            if not isinstance(sensor_snapshot, dict):
                stats["missing_sensor_snapshot_fields"] += 1
            else:
                missing_sensor = [k for k in REQUIRED_SENSOR_SNAPSHOT_FIELDS if k not in sensor_snapshot]
                if missing_sensor:
                    stats["missing_sensor_snapshot_fields"] += 1

            for obj in objects:
                stats["objects"] += 1
                missing_obj = [k for k in REQUIRED_OBJECT_FIELDS if k not in obj]
                if missing_obj:
                    stats["missing_object_fields"] += 1
                    continue

                bbox = obj.get("bbox")
                if not isinstance(bbox, dict):
                    stats["missing_bbox_fields"] += 1
                    continue

                missing_bbox = [k for k in REQUIRED_BBOX_FIELDS if k not in bbox]
                if missing_bbox:
                    stats["missing_bbox_fields"] += 1

                if bool(obj.get("track_confirmed", False)):
                    stats["track_confirmed"] += 1

                distance = obj.get("distance_m", -1)
                if isinstance(distance, (int, float)) and distance >= 0:
                    stats["valid_distance"] += 1

    return stats


def merge_stats(items: List[Dict[str, int]]) -> Dict[str, int]:
    out: Dict[str, int] = {}
    for item in items:
        for k, v in item.items():
            out[k] = out.get(k, 0) + v
    return out


def print_summary(name: str, stats: Dict[str, int]) -> None:
    frames = max(1, stats.get("frames", 0))
    objects = max(1, stats.get("objects", 0))
    print(f"[{name}]")
    print(f"frames={stats.get('frames', 0)} objects={stats.get('objects', 0)}")
    print(
        "errors: "
        f"json={stats.get('json_errors', 0)} "
        f"frame_fields={stats.get('missing_frame_fields', 0)} "
        f"sensor_snapshot={stats.get('missing_sensor_snapshot_fields', 0)} "
        f"object_fields={stats.get('missing_object_fields', 0)} "
        f"bbox_fields={stats.get('missing_bbox_fields', 0)}"
    )
    print(
        "quality: "
        f"confirmed_ratio={stats.get('track_confirmed', 0) / objects:.3f} "
        f"valid_distance_ratio={stats.get('valid_distance', 0) / objects:.3f}"
    )
    print(
        "sanity: "
        f"objects_per_frame={stats.get('objects', 0) / frames:.3f}"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Validate RK3588 pseudo label JSONL files")
    parser.add_argument("path", nargs="?", default="", help="Single JSONL file path")
    parser.add_argument(
        "--glob",
        default="",
        help="Glob pattern for sharded files, e.g. '/tmp/rk3588_pseudo_labels.jsonl*'",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.path and not args.glob:
        print("error: provide a file path or --glob pattern")
        return 2

    pattern = args.glob if args.glob else ""
    paths = list(iter_paths(args.path, pattern))
    if not paths:
        print("error: no input files found")
        return 2

    all_stats: List[Dict[str, int]] = []
    for p in paths:
        s = validate_file(p)
        print_summary(str(p), s)
        all_stats.append(s)

    if len(all_stats) > 1:
        print_summary("TOTAL", merge_stats(all_stats))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
