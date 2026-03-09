# Update Notes - Mathematical Foundation Implementation

Date: 2026-02-26
System: Motion Capture Stereo Biomechanics Engine

## Update - 2026-02-27 (Dual-Camera Persistence & Dashboard)

- Added dual-stream metric computation/state handling in master mode (local + remote camera streams).
- Updated master dashboard rendering to show both local (Mac) and remote (Windows) live biometrics.
- Extended synchronized recording path to archive raw frames for both local and remote cameras when enabled.
- Hardened remote payload serialization + DB fallback path so `PC2` rows persist/export reliably.
- Removed hardcoded remote camera ID assumptions in master GUI/save path (dynamic remote camera ID resolution).
- Suppressed report spectrogram `log10` divide-by-zero warning by using linear magnitude scaling.
- Documentation updated for `--remote` launcher alias, CSV source validation, and VS Code SQLite viewing.

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
