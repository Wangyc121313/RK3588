#!/usr/bin/env python3
"""Evaluate traditional fusion baseline from pseudo-label JSONL.

Computes per-class and global metrics for distance fusion quality.
Designed as the baseline for comparison against an MLP-augmented fusion head.

Usage:
  python3 tools/diagnostics/evaluate_fusion_baseline.py /tmp/rk3588_pseudo_labels.jsonl
  python3 tools/diagnostics/evaluate_fusion_baseline.py --glob '/tmp/rk3588_pseudo_labels.jsonl*'
  python3 tools/diagnostics/evaluate_fusion_baseline.py --glob '...' --output report.json
"""

import argparse
import glob
import json
import math
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

def iter_paths(path: str, pattern: str) -> List[Path]:
    paths: List[Path] = []
    if path:
        p = Path(path)
        if p.exists():
            paths.append(p)
    for matched in sorted(glob.glob(pattern)):
        mp = Path(matched)
        if mp.is_file():
            paths.append(mp)
    return sorted(set(paths))


def load_objects(paths: List[Path]) -> List[Dict[str, Any]]:
    """Load all detection objects across files, keeping frame-level context."""
    records: List[Dict[str, Any]] = []
    for p in paths:
        with p.open("r", encoding="utf-8") as f:
            for raw in f:
                text = raw.strip()
                if not text:
                    continue
                try:
                    row = json.loads(text)
                except json.JSONDecodeError:
                    continue
                frame_id = row.get("frame_id", 0)
                lidar_matched = row.get("lidar_matched", False)
                lidar_delta_ms = row.get("lidar_delta_ms", 0)
                for obj in row.get("objects", []):
                    obj["_frame_id"] = frame_id
                    obj["_lidar_matched"] = lidar_matched
                    obj["_lidar_delta_ms"] = lidar_delta_ms
                    records.append(obj)
    return records


# ---------------------------------------------------------------------------
# Per-object metrics
# ---------------------------------------------------------------------------

def obj_has_distance(obj: Dict[str, Any]) -> bool:
    d = obj.get("distance_m", -1)
    return isinstance(d, (int, float)) and d >= 0


def obj_has_fusion(obj: Dict[str, Any]) -> bool:
    return isinstance(obj.get("fusion"), dict)


# ---------------------------------------------------------------------------
# Aggregation helpers
# ---------------------------------------------------------------------------

def percentile(sorted_vals: List[float], p: float) -> float:
    if not sorted_vals:
        return float("nan")
    idx = (len(sorted_vals) - 1) * p / 100.0
    lo = int(math.floor(idx))
    hi = int(math.ceil(idx))
    if lo == hi:
        return sorted_vals[lo]
    frac = idx - lo
    return sorted_vals[lo] * (1.0 - frac) + sorted_vals[hi] * frac


def stats_1d(vals: List[float]) -> Dict[str, float]:
    if not vals:
        return {"count": 0, "mean": float("nan"), "std": float("nan"),
                "median": float("nan"), "p5": float("nan"), "p95": float("nan")}
    n = len(vals)
    mean = sum(vals) / n
    var = sum((v - mean) ** 2 for v in vals) / n
    s = sorted(vals)
    return {
        "count": n,
        "mean": mean,
        "std": math.sqrt(var),
        "median": percentile(s, 50),
        "p5": percentile(s, 5),
        "p95": percentile(s, 95),
    }


def safe_div(num: float, den: float) -> float:
    return num / den if den > 0 else float("nan")


# ---------------------------------------------------------------------------
# Main evaluation
# ---------------------------------------------------------------------------

def evaluate(records: List[Dict[str, Any]]) -> Dict[str, Any]:
    # ---- global accumulators ----
    total = 0
    with_distance = 0
    with_fusion = 0
    fallback_count = 0
    rejected_count = 0
    smoothed_count = 0

    raw_distances: List[float] = []
    final_distances: List[float] = []
    delta_distances: List[float] = []          # |smoothed - raw|
    candidate_points: List[float] = []
    cluster_points: List[float] = []
    cluster_scores: List[float] = []

    # ---- per-class accumulators ----
    class_data: Dict[str, Dict[str, List[float]]] = defaultdict(
        lambda: {"raw_d": [], "final_d": [], "delta_d": [],
                 "cand_pts": [], "clust_pts": [], "clust_score": []})
    class_counts: Dict[str, Dict[str, int]] = defaultdict(
        lambda: {"total": 0, "with_d": 0, "with_fusion": 0,
                 "fallback": 0, "rejected": 0, "smoothed": 0})

    # ---- per-track accumulators ----
    track_distances: Dict[int, List[Tuple[int, float]]] = defaultdict(list)
    # (frame_id, distance_m)

    for obj in records:
        total += 1
        cls = obj.get("class_name", "unknown")
        class_counts[cls]["total"] += 1

        has_d = obj_has_distance(obj)
        if has_d:
            with_distance += 1
            class_counts[cls]["with_d"] += 1
            d = float(obj["distance_m"])
            final_distances.append(d)
            class_data[cls]["final_d"].append(d)

        fusion = obj.get("fusion")
        if isinstance(fusion, dict):
            with_fusion += 1
            class_counts[cls]["with_fusion"] += 1

            raw = fusion.get("raw_distance_m", -1)
            if raw >= 0:
                raw_distances.append(raw)
                class_data[cls]["raw_d"].append(raw)
                if has_d:
                    delta_distances.append(abs(float(obj["distance_m"]) - raw))
                    class_data[cls]["delta_d"].append(abs(float(obj["distance_m"]) - raw))

            cp = fusion.get("candidate_points", 0)
            candidate_points.append(cp)
            class_data[cls]["cand_pts"].append(cp)

            clp = fusion.get("cluster_points", 0)
            cluster_points.append(clp)
            class_data[cls]["clust_pts"].append(clp)

            cs = fusion.get("cluster_score", 0.0)
            cluster_scores.append(cs)
            class_data[cls]["clust_score"].append(cs)

            if fusion.get("used_fallback", False):
                fallback_count += 1
                class_counts[cls]["fallback"] += 1
            if fusion.get("rejected_by_sanity", False):
                rejected_count += 1
                class_counts[cls]["rejected"] += 1
            if fusion.get("used_temporal_smoothing", False):
                smoothed_count += 1
                class_counts[cls]["smoothed"] += 1

        # track-level
        track_id = obj.get("track_id", -1)
        if isinstance(track_id, int) and track_id >= 0 and has_d:
            frame_id = obj.get("_frame_id", 0)
            if isinstance(frame_id, (int, float)):
                track_distances[track_id].append((int(frame_id), d))

    # ---- cluster score buckets ----
    score_buckets = _bucketize(cluster_scores, [0, 0.5, 2.0, 5.0, 10.0, 20.0])

    # ---- track stability ----
    track_stability = _track_stability(track_distances)

    # ---- per-class summary ----
    per_class = {}
    for cls in sorted(class_counts.keys()):
        cd = class_data[cls]
        cc = class_counts[cls]
        per_class[cls] = {
            "total_objects": cc["total"],
            "distance_coverage": safe_div(cc["with_d"], cc["total"]),
            "fusion_fields_present": safe_div(cc["with_fusion"], cc["total"]),
            "fallback_rate": safe_div(cc["fallback"], max(1, cc["with_fusion"])),
            "rejection_rate": safe_div(cc["rejected"], max(1, cc["with_fusion"])),
            "smoothing_rate": safe_div(cc["smoothed"], max(1, cc["with_fusion"])),
            "raw_distance_m": stats_1d(cd["raw_d"]),
            "final_distance_m": stats_1d(cd["final_d"]),
            "smoothing_delta_m": stats_1d(cd["delta_d"]),
            "candidate_points": stats_1d(cd["cand_pts"]),
            "cluster_points": stats_1d(cd["clust_pts"]),
            "cluster_score": stats_1d(cd["clust_score"]),
        }

    return {
        "file_count": len(records),
        "global": {
            "total_objects": total,
            "distance_coverage": safe_div(with_distance, total),
            "fusion_fields_present": safe_div(with_fusion, total),
            "fallback_rate": safe_div(fallback_count, max(1, with_fusion)),
            "rejection_rate": safe_div(rejected_count, max(1, with_fusion)),
            "smoothing_rate": safe_div(smoothed_count, max(1, with_fusion)),
            "raw_distance_m": stats_1d(raw_distances),
            "final_distance_m": stats_1d(final_distances),
            "smoothing_delta_m": stats_1d(delta_distances),
            "candidate_points": stats_1d(candidate_points),
            "cluster_points": stats_1d(cluster_points),
            "cluster_score": stats_1d(cluster_scores),
            "cluster_score_buckets": score_buckets,
        },
        "per_class": per_class,
        "track_stability": track_stability,
    }


def _bucketize(vals: List[float],
               thresholds: List[float]) -> Dict[str, int]:
    buckets: Dict[str, int] = {}
    for i, lo in enumerate(thresholds):
        hi = thresholds[i + 1] if i + 1 < len(thresholds) else float("inf")
        label = f"[{lo}, {hi})" if hi < float("inf") else f">={lo}"
        buckets[label] = 0
    for v in vals:
        for i, lo in enumerate(thresholds):
            hi = thresholds[i + 1] if i + 1 < len(thresholds) else float("inf")
            if lo <= v < hi:
                label = f"[{lo}, {hi})" if hi < float("inf") else f">={lo}"
                buckets[label] += 1
                break
    return buckets


def _track_stability(
    track_distances: Dict[int, List[Tuple[int, int]]]
) -> Dict[str, float]:
    """Per-track distance std as measure of jitter on static targets."""
    jitters: List[float] = []
    track_lifetimes: List[int] = []
    for tid, entries in track_distances.items():
        entries.sort(key=lambda x: x[0])
        dists = [d for _, d in entries]
        if len(dists) >= 3:
            jitters.append(_std(dists))
        track_lifetimes.append(len(entries))

    return {
        "total_tracks": len(track_distances),
        "mean_track_lifetime_frames": safe_div(sum(track_lifetimes),
                                                float(len(track_lifetimes))),
        "total_track_jitter_samples": len(jitters),
        "track_jitter_std_m": stats_1d(jitters),
    }


def _std(vals: List[float]) -> float:
    if len(vals) < 2:
        return 0.0
    mean = sum(vals) / len(vals)
    return math.sqrt(sum((v - mean) ** 2 for v in vals) / (len(vals) - 1))


# ---------------------------------------------------------------------------
# Output formatting
# ---------------------------------------------------------------------------

def print_report(result: Dict[str, Any]) -> None:
    g = result["global"]
    print("=" * 72)
    print("  Fusion Baseline Report")
    print("=" * 72)

    print(f"\n  objects: {g['total_objects']}")
    print(f"  distance_coverage:       {g['distance_coverage']:.3f}  "
          f"(fraction of detections with valid distance)")
    print(f"  fusion_fields_present:   {g['fusion_fields_present']:.3f}  "
          f"(fraction with diagnostic fields)")
    print(f"  fallback_rate:           {g['fallback_rate']:.3f}  "
          f"(angle-window fallback, not cluster-based)")
    print(f"  rejection_rate:          {g['rejection_rate']:.3f}  "
          f"(failed sanity checks)")
    print(f"  smoothing_rate:          {g['smoothing_rate']:.3f}  "
          f"(used temporal smoothing)")

    _print_stats("raw_distance_m", g["raw_distance_m"], "m")
    _print_stats("final_distance_m", g["final_distance_m"], "m")
    _print_stats("smoothing_delta_m", g["smoothing_delta_m"], "m")
    _print_stats("candidate_points", g["candidate_points"], "pts")
    _print_stats("cluster_points", g["cluster_points"], "pts")
    _print_stats("cluster_score", g["cluster_score"], "")

    print("\n  cluster_score histogram:")
    for bucket, count in g["cluster_score_buckets"].items():
        bar = "#" * max(1, count // max(1, g["total_objects"] // 40 + 1))
        print(f"    {bucket:>12s}  {count:5d}  {bar}")

    # per-class
    print("\n" + "-" * 72)
    print("  Per-Class Breakdown")
    print("-" * 72)
    header = (f"  {'class':<16s} {'total':>6s} {'cov':>6s} {'fallback':>9s} "
              f"{'reject':>7s} {'raw_mean':>9s} {'final_mean':>10s} "
              f"{'cand_pts':>9s} {'clust_pts':>9s}")
    print(header)
    print("  " + "-" * len(header.strip()))
    for cls in sorted(result["per_class"].keys()):
        c = result["per_class"][cls]
        print(
            f"  {cls:<16s} "
            f"{c['total_objects']:6d} "
            f"{c['distance_coverage']:6.3f} "
            f"{c['fallback_rate']:9.3f} "
            f"{c['rejection_rate']:7.3f} "
            f"{c['raw_distance_m']['mean']:9.2f} "
            f"{c['final_distance_m']['mean']:10.2f} "
            f"{c['candidate_points']['mean']:9.1f} "
            f"{c['cluster_points']['mean']:9.1f}"
        )

    # track stability
    ts = result["track_stability"]
    print(f"\n  track_stability:")
    print(f"    total_tracks:             {ts['total_tracks']}")
    print(f"    mean_lifetime_frames:     {ts['mean_track_lifetime_frames']:.1f}")
    jitter_s = ts["track_jitter_std_m"]
    print(f"    jitter_samples:           {jitter_s['count']}")
    print(f"    jitter_mean_std_m:        {jitter_s['mean']:.4f}")
    print(f"    jitter_median_std_m:      {jitter_s['median']:.4f}")
    print(f"    jitter_p95_std_m:         {jitter_s['p95']:.4f}")

    print()


def _print_stats(label: str, s: Dict[str, float], unit: str) -> None:
    unit_str = f" {unit}" if unit else ""
    print(
        f"  {label:<25s}  "
        f"mean={s['mean']:7.3f}{unit_str}  "
        f"std={s['std']:7.3f}{unit_str}  "
        f"p50={s['median']:7.3f}{unit_str}  "
        f"(n={s['count']})"
    )


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Evaluate traditional fusion baseline from pseudo-label JSONL"
    )
    parser.add_argument("path", nargs="?", default="",
                        help="Single JSONL file path")
    parser.add_argument("--glob", default="",
                        help="Glob pattern for sharded files")
    parser.add_argument("--output", default="",
                        help="Optional JSON output path for machine-readable report")
    parser.add_argument("--json", action="store_true",
                        help="Print JSON to stdout instead of human-readable report")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.path and not args.glob:
        print("error: provide a file path or --glob pattern", file=sys.stderr)
        return 2

    paths = iter_paths(args.path, args.glob)
    if not paths:
        print("error: no input files found", file=sys.stderr)
        return 2

    print(f"loading {len(paths)} file(s)...", file=sys.stderr)
    records = load_objects(paths)
    if not records:
        print("error: no objects found in input", file=sys.stderr)
        return 1

    result = evaluate(records)

    if args.output:
        with open(args.output, "w", encoding="utf-8") as f:
            json.dump(result, f, indent=2, default=str)
        print(f"wrote {args.output}", file=sys.stderr)

    if args.json:
        print(json.dumps(result, indent=2, default=str))
    else:
        print_report(result)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
