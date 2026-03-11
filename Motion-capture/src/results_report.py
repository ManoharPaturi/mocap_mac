"""Consolidated evaluation report writer for results.md."""

from __future__ import annotations

import glob
import json
import os
import time
from typing import Any, Dict, Optional


_KEY_METRICS = [
    "pipeline_latency_ms",
    "fps",
    "reprojection_error_px",
    "epipolar_error_px",
    "triangulation_success_rate",
    "network_latency_ms",
    "synchronization_error_ms",
    "packet_loss_rate",
    "cpu_percent",
    "memory_mb",
    "occlusion_recovery_ms",
    "joint_jitter_m",
    "bone_length_variance_m",
    "depth_stability_m",
]

_NOTES = {
    "pipeline_latency_ms": "Real-time range",
    "fps": "Auto-estimated runtime FPS",
    "reprojection_error_px": "Lower is better",
    "epipolar_error_px": "Lower is better; calibration-sensitive",
    "triangulation_success_rate": "Higher is better",
    "network_latency_ms": "Lower is better",
    "synchronization_error_ms": "Lower is better",
    "packet_loss_rate": "Lower is better",
    "cpu_percent": "Process-level CPU usage",
    "memory_mb": "Resident memory footprint",
    "occlusion_recovery_ms": "Lower is better",
    "joint_jitter_m": "Lower is smoother",
    "bone_length_variance_m": "Lower is more stable",
    "depth_stability_m": "Lower is more stable",
}


def _safe_load_json(path: str) -> Optional[Dict[str, Any]]:
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        return data if isinstance(data, dict) else None
    except Exception:
        return None


def _latest_session_aggregate(repo_root: str, output_root: str) -> Optional[str]:
    pattern = os.path.join(repo_root, output_root, "session_*", "evaluation", "aggregate_metrics.json")
    matches = glob.glob(pattern)
    if not matches:
        return None
    matches.sort(key=lambda p: os.path.getmtime(p), reverse=True)
    return matches[0]


def _line_count(path: str) -> int:
    try:
        with open(path, "r", encoding="utf-8") as f:
            return sum(1 for _ in f)
    except Exception:
        return 0


def _rel(repo_root: str, path: str) -> str:
    try:
        return os.path.relpath(path, repo_root).replace("\\", "/")
    except Exception:
        return path


def write_consolidated_results_report(
    repo_root: str,
    output_root: str = "results",
    report_filename: str = "results.md",
) -> str:
    """Create/update a single consolidated markdown report at repo root."""
    repo_root = os.path.abspath(repo_root)
    latest_index_path = os.path.join(repo_root, output_root, "latest_evaluation_artifacts.json")

    latest_index = _safe_load_json(latest_index_path) or {}
    artifacts = latest_index.get("artifacts") if isinstance(latest_index, dict) else {}
    if not isinstance(artifacts, dict):
        artifacts = {}

    agg_from_index = artifacts.get("aggregate_json")
    if isinstance(agg_from_index, str):
        agg_candidate = os.path.join(repo_root, agg_from_index)
        aggregate_path = agg_candidate if os.path.exists(agg_candidate) else None
    else:
        aggregate_path = None

    if aggregate_path is None:
        aggregate_path = _latest_session_aggregate(repo_root, output_root)
    if aggregate_path is None:
        raise FileNotFoundError("No aggregate_metrics.json found under results/session_*/evaluation/")

    aggregate = _safe_load_json(aggregate_path)
    if not aggregate:
        raise RuntimeError(f"Failed to parse aggregate metrics JSON: {aggregate_path}")

    session_label = str(aggregate.get("session_label") or "unknown")
    eval_dir = os.path.dirname(aggregate_path)

    per_frame_csv = os.path.join(eval_dir, "per_frame_metrics.csv")
    eval_md = os.path.join(eval_dir, "evaluation_table.md")
    latency_png = os.path.join(eval_dir, "latency_kde.png")
    bone_png = os.path.join(eval_dir, "bone_variance_line.png")
    jitter_png = os.path.join(eval_dir, "jitter_scatter.png")

    csv_lines = _line_count(per_frame_csv)
    data_rows = max(0, csv_lines - 1)
    metrics = aggregate.get("metrics") if isinstance(aggregate.get("metrics"), dict) else {}

    lines = []
    lines.append("# Motion Capture Results Consolidated Report")
    lines.append("")
    lines.append(f"**Date:** {time.strftime('%d %B %Y')}")
    lines.append("**Project:** `Motion-capture`")
    lines.append("**Purpose:** One-place summary of evaluation outputs, key metrics, and figures.")
    lines.append("")
    lines.append("---")
    lines.append("")

    lines.append("## 1) Session Used For Main Results")
    lines.append("")
    lines.append("Primary evaluation session:")
    lines.append(f"- `{session_label}`")
    lines.append("")
    lines.append("Evaluation artifact folder:")
    lines.append(f"- `{_rel(repo_root, eval_dir)}/`")
    lines.append("")
    lines.append("Artifacts generated:")
    lines.append(f"- `{_rel(repo_root, per_frame_csv)}`")
    lines.append(f"- `{_rel(repo_root, aggregate_path)}`")
    lines.append(f"- `{_rel(repo_root, eval_md)}`")
    lines.append(f"- `{_rel(repo_root, latency_png)}`")
    lines.append(f"- `{_rel(repo_root, bone_png)}`")
    lines.append(f"- `{_rel(repo_root, jitter_png)}`")
    lines.append("")
    lines.append("Row count check:")
    lines.append(f"- `per_frame_metrics.csv`: `{csv_lines}` lines (`1` header + `{data_rows}` data rows)")
    lines.append("")
    lines.append("---")
    lines.append("")

    lines.append("## 2) Key Aggregate Metrics")
    lines.append("")
    lines.append(f"- Frames evaluated: `{int(aggregate.get('frames_evaluated', 0) or 0)}`")
    lines.append(f"- Calibration RMS (px): `{float(aggregate.get('calibration_rms_px', 0.0) or 0.0):.4f}`")
    gt_src = aggregate.get("ground_truth_dataset")
    lines.append(f"- Ground truth dataset: `{gt_src if gt_src else 'null'}`")
    lines.append("")
    lines.append("| Metric | Mean | P95 | Max | Notes |")
    lines.append("|---|---:|---:|---:|---|")

    for key in _KEY_METRICS:
        m = metrics.get(key, {}) if isinstance(metrics, dict) else {}
        try:
            mean = float(m.get("mean", 0.0))
            p95 = float(m.get("p95", 0.0))
            maxv = float(m.get("max", 0.0))
        except Exception:
            mean, p95, maxv = 0.0, 0.0, 0.0

        note = _NOTES.get(key, "")
        if key == "fps" and mean > 500.0:
            note = "Potentially inflated; validate with latest patched run"

        lines.append(f"| `{key}` | {mean:.4f} | {p95:.4f} | {maxv:.4f} | {note} |")

    lines.append("")
    lines.append("---")
    lines.append("")

    lines.append("## 3) Figures (Embedded)")
    lines.append("")
    lines.append("### 3.1 Pipeline Latency Distribution")
    lines.append("")
    lines.append(f"![Pipeline Latency KDE]({_rel(repo_root, latency_png)})")
    lines.append("")
    lines.append("### 3.2 Bone Length Variance Over Time")
    lines.append("")
    lines.append(f"![Bone Variance Line]({_rel(repo_root, bone_png)})")
    lines.append("")
    lines.append("### 3.3 Joint Jitter Scatter")
    lines.append("")
    lines.append(f"![Joint Jitter Scatter]({_rel(repo_root, jitter_png)})")
    lines.append("")
    lines.append("---")
    lines.append("")

    lines.append("## 4) Latest Pointer State")
    lines.append("")
    lines.append(f"- Pointer file: `{_rel(repo_root, latest_index_path)}`")
    idx_label = latest_index.get("session_label") if isinstance(latest_index, dict) else None
    idx_frames = latest_index.get("frames_evaluated") if isinstance(latest_index, dict) else None
    lines.append(f"- Indexed session label: `{idx_label if idx_label else 'unknown'}`")
    lines.append(f"- Indexed frames evaluated: `{idx_frames if idx_frames is not None else 'unknown'}`")
    lines.append(f"- Report source aggregate: `{_rel(repo_root, aggregate_path)}`")

    out_path = os.path.join(repo_root, report_filename)
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

    return out_path
