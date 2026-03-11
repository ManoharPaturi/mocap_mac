# Motion Capture Results Consolidated Report

**Date:** 11 March 2026
**Project:** `Motion-capture`
**Purpose:** One-place summary of evaluation outputs, key metrics, and figures.

---

## 1) Session Used For Main Results

Primary evaluation session:
- `live_20260311_170353`

Evaluation artifact folder:
- `results/live_20260311_170353/evaluation/`

Artifacts generated:
- `results/live_20260311_170353/evaluation/per_frame_metrics.csv`
- `results/live_20260311_170353/evaluation/aggregate_metrics.json`
- `results/live_20260311_170353/evaluation/evaluation_table.md`
- `results/live_20260311_170353/evaluation/latency_kde.png`
- `results/live_20260311_170353/evaluation/bone_variance_line.png`
- `results/live_20260311_170353/evaluation/jitter_scatter.png`

Row count check:
- `per_frame_metrics.csv`: `285` lines (`1` header + `284` data rows)

---

## 2) Key Aggregate Metrics

- Frames evaluated: `240`
- Calibration RMS (px): `0.0000`
- Ground truth dataset: `null`

| Metric | Mean | P95 | Max | Notes |
|---|---:|---:|---:|---|
| `pipeline_latency_ms` | 84.3697 | 138.1050 | 273.4207 | Real-time range |
| `fps` | 14.4726 | 29.9103 | 33.1423 | Auto-estimated runtime FPS |
| `reprojection_error_px` | 2.6447 | 10.5660 | 13.8929 | Lower is better |
| `epipolar_error_px` | 218.8284 | 264.0380 | 323.4209 | Lower is better; calibration-sensitive |
| `triangulation_success_rate` | 0.0098 | 0.0303 | 0.0303 | Higher is better |
| `network_latency_ms` | 0.8735 | 1.2200 | 57.1881 | Lower is better |
| `synchronization_error_ms` | 20.7314 | 46.9222 | 115.7171 | Lower is better |
| `packet_loss_rate` | 0.0883 | 0.1538 | 0.1935 | Lower is better |
| `cpu_percent` | 127.4979 | 163.0000 | 190.1000 | Process-level CPU usage |
| `memory_mb` | 518.6257 | 579.3594 | 604.0469 | Resident memory footprint |
| `occlusion_recovery_ms` | 0.0000 | 0.0000 | 0.0000 | Lower is better |
| `joint_jitter_m` | 0.0648 | 0.2609 | 1.0929 | Lower is smoother |
| `bone_length_variance_m` | 0.6025 | 1.7828 | 1.9096 | Lower is more stable |
| `depth_stability_m` | 0.0080 | 0.0199 | 0.0213 | Lower is more stable |

---

## 3) Figures (Embedded)

### 3.1 Pipeline Latency Distribution

![Pipeline Latency KDE](results/live_20260311_170353/evaluation/latency_kde.png)

### 3.2 Bone Length Variance Over Time

![Bone Variance Line](results/live_20260311_170353/evaluation/bone_variance_line.png)

### 3.3 Joint Jitter Scatter

![Joint Jitter Scatter](results/live_20260311_170353/evaluation/jitter_scatter.png)

---

## 4) Latest Pointer State

- Pointer file: `results/latest_evaluation_artifacts.json`
- Indexed session label: `live_20260311_170353`
- Indexed frames evaluated: `240`
- Report source aggregate: `results/live_20260311_170353/evaluation/aggregate_metrics.json`
