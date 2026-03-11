# Architecture Change Tracker

Tracks architecture/documentation changes made in this repository.

## 2026-03-02

### Entry 001
- **Scope:** Align architecture docs to implemented runtime pipeline.
- **Source of truth used:** Runtime code paths in `main_gui.py`, `src/master_coordinator.py`, `src/database.py`, and `config.py`.
- **Changes made:**
  - Rewrote `SYSTEM_ARCHITECTURE.md` to describe only implemented behavior.
  - Enforced canonical pipeline sequence:
    - Capture (Front)
    - Capture (Right)
    - Time Sync
    - Matched Frame Pair
    - Triangulate
    - 3D Frame Object
    - Kinematics
    - Store
    - Display
  - Added `ARCHITECTURE_CHANGE_TRACKER.md` (this file) for ongoing tracking.
- **Notes:** `CHANGES.md` is treated as Windows/server-side implementation reference for coordination.

### Entry 002
- **Scope:** Workspace cleanup.
- **Changes made:**
  - Removed duplicate folder `Motion-capture-vs2` to avoid repo ambiguity.
- **Notes:** Active working repo remains `Motion-capture`.

### Entry 003
- **Scope:** Repository structure cleanup aligned with Windows-system layout.
- **Changes made:**
  - Created root folders: `docs/`, `data/`, `tools/`, `config/`.
  - Moved documentation files from root to `docs/`:
    - `SETUP.md`, `DOCUMENTATION.md`, `IMPLEMENTATION_PROGRESS.md`, `UPDATE_NOTES.md`, `COORDINATE_SYSTEM_SPEC.md`
  - Moved sample/session data from root to `data/`:
    - `mocap_*.csv`, `mocap_data.db`, `dashboard_session_*.html`
  - Moved launcher implementation to `tools/launch_multi_camera.py`.
  - Added root compatibility shim `launch_multi_camera.py` to preserve existing command usage.
  - Updated README paths to new `docs/` and `data/` locations.
- **Notes:** `CHANGES.md` from Windows remains the coordination reference snapshot.

### Entry 004
- **Scope:** Post-cleanup documentation path alignment.
- **Changes made:**
  - Updated `docs/DOCUMENTATION.md` file-path references to new layout (`docs/` and `data/`).
  - Updated SQLite DB reference from `mocap_data.db` to `data/mocap_data.db`.
  - Updated concise architecture cross-reference to `../SYSTEM_ARCHITECTURE.md`.

### Entry 005
- **Scope:** Dependency completeness update.
- **Changes made:**
  - Updated `requirements.txt` to include runtime imports used by code paths:
    - `Pillow` (Tkinter image display path in `main_gui.py`)
    - `torch` (GPU/runtime checks and triangulation support)
    - `torchvision` (GPU JPEG decode helpers in `main_gui.py`)

## 2026-03-10

### Entry 006
- **Scope:** Live video feed buffering elimination and async persistence.
- **Source of truth:** `src/camera_server.py`, `main_gui.py`, `src/master_coordinator.py`.
- **Changes made:**
  - `camera_server.py`: `SNDHWM` → 1, `CONFLATE` → 1 (latest-only ZMQ transport).
  - `camera_server.py`: face/hand/2D-pose landmark blobs stripped from packet; only compact 2D `landmarks` + `pose_world_landmarks` + JPEG transmitted.
  - `camera_server.py`: `capture_timestamp_ns` (frame grab time) separated from `timestamp` (actual send time).
  - `main_gui.py`: `_master_result_queue` maxsize reduced from 20 → 2.
  - `main_gui.py`: async `_db_save_queue` + `_db_save_worker` thread added; SQLite saves decoupled from capture hot path.
  - `main_gui.py`: `LiveVisualizer3D` and `Visualizer3D` removed from GUI startup (reduces RAM ~80–120 MB).
  - `main_gui.py`: FPS label update moved off video thread to prevent cross-thread Tk access.
  - `src/master_coordinator.py`: passive clock offset fallback, per-camera FPS tracking, adaptive sync threshold bounds (`SYNC_THRESHOLD_MIN_MS`, `SYNC_THRESHOLD_MAX_MS`).
- **Notes:** These changes correspond to Session 4 in `CHANGES.md`.

## 2026-03-11

### Entry 007
- **Scope:** Evaluation pipeline and research reporting infrastructure.
- **Changes made:**
  - Added `src/evaluation_pipeline.py` (`MotionCaptureEvaluationPipeline`):
    - per-frame metrics CSV, aggregate JSON, evaluation_table.md.
    - figures: `latency_kde.png`, `bone_variance_line.png`, `jitter_scatter.png`.
    - ground-truth MPJPE support, calibration RMS tracking, epipolar error.
  - Added `src/results_report.py` (`write_consolidated_results_report`):
    - scans `results/session_*/evaluation/aggregate_metrics.json`.
    - writes `results.md` at repo root with mean/P95/max table + figure embeds.
  - Added `tools/generate_results_report.py`: standalone CLI regenerator.
  - Added `wiretap.py`: ZMQ SUB packet inspector diagnostic.
  - Added `check.py`: SQLite session table sanity checker.
  - `SYSTEM_ARCHITECTURE.md` bumped to VS7.1.
- **Notes:** Evaluation pipeline is wired into `main_gui.py` session lifecycle.

