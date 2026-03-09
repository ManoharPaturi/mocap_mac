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
