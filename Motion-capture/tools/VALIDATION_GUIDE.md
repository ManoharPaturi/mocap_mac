# Validation Guide

## Quick Start

1. Run capture in GUI (`python main_gui.py`) and record a session.
2. Validate session:
   - `python tools/validate_session.py --session <SESSION_ID> --threshold 2.0`
3. Generate reports:
   - `python tools/compare_angles.py --session <SESSION_ID> --format all`
4. Export artifacts:
   - `python tools/export_validation_artifacts.py --session <SESSION_ID> --include-all`

## Tools

- `tools/validate_session.py`
  - Recomputes angles offline from `joints_3d`
  - Compares with real-time `kinematics_3d`
  - Writes Layer-4 summary into `validation_runs`
  - Writes per-angle rows into `validation_angle_comparison`

- `tools/compare_angles.py`
  - Reads latest `validation_runs` entry for session
  - Generates CSV / Markdown / JSON reports
  - Records generated files in `validation_artifacts`

- `tools/export_validation_artifacts.py`
  - Exports Layer-2 3D joints (`3d_coordinates_*.csv`)
  - Optionally exports Layer-1 2D landmarks (`landmarks_*.csv`)
  - Optionally exports latest validation comparison + session metadata

- `tools/test_validation_system.py`
  - Smoke-checks required tables and DB API methods

## Expected Outputs

Under `results/validation/`:

- `validation_report_<SESSION_ID>_<timestamp>.json`
- `comparison_<SESSION_ID>.csv`
- `report_<SESSION_ID>.md`
- `stats_<SESSION_ID>.json`
- `3d_coordinates_<SESSION_ID>.csv`
- `landmarks_<camera>_<SESSION_ID>.csv` (optional)
- `angle_comparison_<SESSION_ID>.csv` (when `--include-all` and validation exists)
- `session_metadata_<SESSION_ID>.json` (when `--include-all`)

## Troubleshooting

- **No frames found**: verify `frames` has rows for the session.
- **No comparable angle rows**: ensure both `joints_3d` and `kinematics_3d` are populated for the same frames.
- **No validation run for compare/export**: run `validate_session.py` first.
