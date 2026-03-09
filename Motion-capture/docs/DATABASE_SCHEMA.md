# 4-Layer Database Schema

## Layer Model

- **Layer 0 (Timeline)**: `frames`
- **Layer 1 (RAW)**: `raw_landmarks_2d`
- **Layer 2 (Reconstruction)**: `joints_3d`
- **Layer 3 (Derived)**: `kinematics_3d`, `kinematics_2d`, `velocities`, `accelerations`
- **Layer 4 (Validation)**: `validation_runs`, `validation_angle_comparison`, `validation_artifacts`

## Core Tables

### `sessions`
- `id` TEXT PRIMARY KEY
- `start_time` TEXT
- `table_name` TEXT
- `recording_mode` TEXT (`stereo` | `single`)

### `frames` (Layer 0)
- `id` INTEGER PRIMARY KEY
- `session_id` TEXT
- `timestamp_ms` REAL
- `frame_index` INTEGER
- `num_cameras` INTEGER
- `created_at` TEXT

### `raw_landmarks_2d` (Layer 1)
- `id` INTEGER PRIMARY KEY
- `session_id` TEXT
- `frame_id` INTEGER
- `camera_id` TEXT
- `landmark_id` INTEGER
- `x`, `y`, `z` REAL
- `visibility` REAL
- `timestamp_ms` REAL
- `source` TEXT

### `joints_3d` (Layer 2)
- `id` INTEGER PRIMARY KEY
- `session_id` TEXT
- `frame_id` INTEGER
- `joint_id` INTEGER
- `x`, `y`, `z` REAL
- `confidence` REAL
- `reprojection_error_px` REAL
- `timestamp_ms` REAL
- `triangulation_version` TEXT
- `algorithm` TEXT
- `source` TEXT

### `kinematics_3d` (Layer 3)
- `id` INTEGER PRIMARY KEY
- `session_id` TEXT
- `frame_id` INTEGER
- `angle_name` TEXT
- `angle_deg` REAL
- `timestamp_ms` REAL
- `smoothing_version` TEXT
- `triangulation_version` TEXT
- `source` TEXT
- `confidence` REAL
- `raw_angle_degrees` REAL

### `kinematics_2d` (Layer 3 fallback)
- `id` INTEGER PRIMARY KEY
- `session_id` TEXT
- `frame_id` INTEGER
- `camera_id` TEXT
- `angle_name` TEXT
- `angle_deg` REAL
- `timestamp_ms` REAL
- `smoothing_version` TEXT
- `source` TEXT

### `velocities`, `accelerations` (Layer 3 derivatives)
- `id` INTEGER PRIMARY KEY
- `session_id` TEXT
- `frame_id` INTEGER
- `metric_name` TEXT
- `value` REAL
- `timestamp_ms` REAL
- `source` TEXT

### `validation_runs` (Layer 4)
- `id` INTEGER PRIMARY KEY
- `session_id` TEXT
- `created_at` TEXT
- `num_frames` INTEGER
- `num_angles` INTEGER
- `pass_rate` REAL
- `mean_error` REAL
- `max_error` REAL
- `threshold` REAL

### `validation_angle_comparison` (Layer 4)
- `id` INTEGER PRIMARY KEY
- `validation_run_id` INTEGER
- `session_id` TEXT
- `frame_id` INTEGER
- `angle_name` TEXT
- `realtime_angle` REAL
- `offline_angle` REAL
- `error_deg` REAL
- `pass_flag` INTEGER

### `validation_artifacts` (Layer 4)
- `id` INTEGER PRIMARY KEY
- `validation_run_id` INTEGER
- `session_id` TEXT
- `artifact_type` TEXT
- `file_path` TEXT
- `metadata_json` TEXT
- `created_at` TEXT

## Runtime Population Path

1. `save_frame_metadata()` → Layer 0
2. `save_raw_landmarks()` per camera → Layer 1
3. `save_joints_3d()` → Layer 2
4. `save_kinematics_3d()` → Layer 3
5. `save_validation_run()` + validation comparison/artifact inserts → Layer 4
