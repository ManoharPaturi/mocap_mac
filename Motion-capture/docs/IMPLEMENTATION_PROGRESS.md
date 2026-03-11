# Implementation Progress

Date: 2026-02-26 (last updated: 2026-03-11)

## Status Summary (11 March 2026)

- Foundation alignment complete for coordinate and kinematic math conventions.
- Core stereo path already present locally (sync, triangulation, 3D filtering).
- New dedicated kinematics engine and validation scripts added in this workspace.
- Evaluation pipeline (`src/evaluation_pipeline.py`) fully wired into capture session lifecycle.
- Live video buffering eliminated; async DB worker added; network packet trimmed.
- `results.md` auto-generated from session evaluation artifacts.

## Pipeline Status

1. Capture (2D + confidence + timestamp): ✅ Implemented (CAP_BUFFERSIZE=1 flush added)
2. Synchronization: ✅ Implemented (passive clock fallback + bounded adaptive threshold)
3. Undistortion and projection handling: ✅ Implemented
4. Triangulation: ✅ Implemented
5. Confidence filtering / reliability tagging: ✅ Implemented
6. 3D filtering (1-Euro): ✅ Implemented
7. Kinematics engine (v/a/angles/angular velocity): ✅ Implemented
8. Dashboard full kinematics panels: Partial
9. Data logging for new flattened kinematics fields: ✅ Implemented (async DB worker)
10. Validation protocol automation: ✅ Implemented (script level)
11. Evaluation pipeline (per-frame metrics + aggregate reports): ✅ Implemented
12. Consolidated results report (`results.md` + figures): ✅ Implemented

## Immediate Follow-ups

1. Tune sync thresholds after empirical jitter/angle/distance results.
2. Run validation scripts on real capture sessions and update results.
3. Restore LiveVisualizer3D as optional side-window (currently removed from hot path).
4. Add CI check to fail when evaluation CSV is header-only.
