# Update Notes — 11 March 2026 (Buffering + Evaluation Pipeline)

Date: 2026-03-11
System: Motion Capture Stereo Biomechanics Engine — Mac Master Node

## Completed in This Pass

- **Live video buffering eliminated**: ZMQ PUB socket now uses `SNDHWM=1` and
  `CONFLATE=1`; `CAP_BUFFERSIZE=1` set on camera capture to flush stale driver
  frames. Combined effect: master always processes the live camera frame, not a
  buffered one.
- **Async DB worker added**: `_db_save_queue` + `_db_save_worker` thread in
  `main_gui.py` decouple SQLite writes from the capture hot path. Save-path
  tracing (`_save_trace`) helps diagnose lost PC2/3D data.
- **Result queue tightened**: `_master_result_queue` maxsize reduced from 20 → 2.
- **Network packet trimmed**: face/hand/2D-pose landmark blobs removed from
  transmitted packet; only compact 2D landmarks + pose_world_landmarks + JPEG
  are sent. Reduces per-frame bandwidth by ~8–25 KB.
- **Evaluation pipeline**: `src/evaluation_pipeline.py` added. Records per-frame
  metrics (latency, FPS, reprojection error, epipolar error, bone variance,
  joint jitter, occlusion recovery, triangulation success rate, CPU/RAM). Writes
  `per_frame_metrics.csv`, `aggregate_metrics.json`, figures, and
  `evaluation_table.md` per session.
- **Consolidated results report**: `src/results_report.py` + `results.md` at
  repo root. Updated automatically after each session.
- **Standalone report tool**: `tools/generate_results_report.py`.
- **Diagnostics**: `wiretap.py` (ZMQ packet inspector) and `check.py` (SQLite
  sanity check).
- **RAM reduction**: `LiveVisualizer3D` and `Visualizer3D` removed from GUI
  startup (~80–120 MB freed).
- **Passive clock fallback**: `master_coordinator.py` derives offset from first
  packet timestamps when active clock sync samples are insufficient.
- **Adaptive sync threshold bounds**: `SYNC_THRESHOLD_MIN_MS` /
  `SYNC_THRESHOLD_MAX_MS` added to `config.py`; dynamic threshold clamped.
- **SYSTEM_ARCHITECTURE.md** bumped to VS7.1.

## Notes

- `results.md` is auto-generated; do not edit manually.
- `wiretap.py` hardcodes the Windows IP (`10.28.109.228`) — update if the
  Windows node changes address.

---
# Update Notes - Mathematical Foundation Implementation

Date: 2026-02-26
System: Motion Capture Stereo Biomechanics Engine

## Completed in Local Coordination Pass

- Added locked coordinate system spec document.
- Added implementation progress tracker.
- Added dedicated `src/kinematics_engine.py` for disciplined derivative and angle math.
- Added validation scripts:
  - `tests/test_static_jitter.py`
  - `tests/test_known_angle.py`
  - `tests/test_known_distance.py`
- Integrated master pipeline to use dedicated kinematics engine while preserving legacy output keys.

## Notes

- Existing local code already had major overlap (axis transform, confidence gating, 3D filtering, triangulation, synchronization).
- This pass focuses on cross-system consistency and reducing drift between Windows and Mac workspaces.
