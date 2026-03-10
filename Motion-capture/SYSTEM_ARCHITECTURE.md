# Motion Capture System Architecture (Implementation-Accurate)

**Version:** VS6 (10 March 2026 — post bandwidth-optimisation update)
**Last Updated:** 10 March 2026  
**Scope:** This file describes only behavior currently implemented in this repository.

---

## 1) Runtime Modes

Configured in `config.py` with `MULTI_CAMERA_MODE`:
- `single`: local camera pipeline only
- `server`: local camera pipeline + network broadcast of frame payloads
- `master`: local camera pipeline + remote receive + synchronization + 3D fusion

This document focuses on `master` mode because that is where multi-camera architecture runs.

---

## 2) Two-Node Setup

```
Windows Node (server)                Mac Node (master)
─────────────────────                ──────────────────────────────────────
NETWORK_CAMERA_ID = 'cam_0'          MULTI_CAMERA_MODE = 'master'
MASTER_IP = '10.137.227.228'         NUM_CAMERAS = 2
DISCOVERY_PORT = 6000                local camera → frame_buffers['local_cam']
DATA_PORT = 6001                     ZMQ SUB ← cam_0 on DATA_PORT 6001
CLOCK_SYNC_PORT = 6003               ClockSync REQ/REP on port 6003
FEEDBACK_PORT = 6002                 Quality feedback SUB on port 6002
NETWORK_PROTOCOL = 'tcp'
ZMQ PUB SNDHWM = 5, CONFLATE = 0    ZMQ SUB RCVHWM = 5, CONFLATE = 0
```

---

## 3) Canonical Master Pipeline (Current Runtime)

```
Windows Camera → CameraServer.send_frame_data()
      │   ZMQ PUB tcp  (SNDHWM=5, CONFLATE=0)
      ▼
Mac _data_receiver() [background thread]
      │   ZMQ SUB (RCVHWM=5, CONFLATE=0) — all frames received, not drain-to-latest
      ▼
_process_frame_data()
      │   clock-offset correction → compact_results_for_sync() → frame_buffers['cam_0'] (deque maxlen=30)
      │   JPEG stored separately in _latest_camera_jpeg (not in FrameData)
      │
Mac video_loop [capture thread]
      │   frame_buffers['local_cam'] (deque maxlen=30)
      ▼
get_synchronized_batch()   [nearest-frame matching]
      │   anchor = slowest camera's newest frame
      │   accepts each camera's closest frame within SYNC_TIME_THRESHOLD_MS=200ms
      ▼
_master_compute_worker [background thread]
      │   get_synced_3d_pose() → triangulate → kinematics
      │   result → _master_result_queue (maxsize=20)
      ▼
_drain_master_results [main thread]
      │   if stereo sync:  save_synced_frame(pc1, pc2, pose_3d)
      │   else (fallback): save_synced_frame(pc1, latest_cam0_buffer, mono_pose_3d)
      ▼
database._worker_loop() → SQLite INSERT (batch 50)
```

---

## 4) Network Packet Format (Windows → Mac)

Each frame sent by `CameraServer.send_frame_data()` over ZMQ PUB:

```python
{
    'type':           'frame_data',
    'schema_version': MESSAGE_SCHEMA_VERSION,
    'camera_id':      self.camera_id,          # 'cam_0'
    'frame_number':   int,
    'timestamp':      int,                     # epoch nanoseconds (time.time_ns)
    'calibration_id': CALIBRATION_ID,
    'capture_fps':    FPS,
    'landmarks':      [{x, y, conf}, ...],     # 33-joint compact 2D (normalized 0–1)
    'results':        {'pose_world_landmarks': [...]},  # Tier-3 fallback only
    'gpu_compute':    {...} or None,
    'frame_jpeg':     bytes,                   # JPEG at NETWORK_JPEG_QUALITY=35
}
```

**Landmark field breakdown:**

| Field | Content | Used for |
|---|---|---|
| `landmarks` | `[{x, y, conf}]` × 33 joints, normalized | Primary triangulation input (Tier 1 & 2) |
| `results.pose_world_landmarks` | `[{x,y,z,vis}]` × 33 joints, metric (hip-relative) | Tier-3 fallback only |
| `frame_jpeg` | JPEG bytes (640×360, Q=35) | Live video display; not stored in FrameData |

**What is NOT sent (stripped to reduce bandwidth):**
- `pose_landmarks` (full x/y/z — redundant with `landmarks`)
- `face_landmarks` (478 landmarks — not consumed by coordinator)
- `hand_landmarks` (up to 42 landmarks — not consumed by coordinator)

**Reception on Mac:**
- `packet_landmarks` is merged into `results` before compaction.
- JPEG is stored separately in `_latest_camera_jpeg[camera_id]` and never enters `FrameData.results`.

---

## 5) Stage Behavior (Exactly What Code Executes)

### Stage 1 — Capture (local_cam / cam_0)

- Local capture is read in `main_gui.py` video loop; frame appended to `coordinator.frame_buffers['local_cam']`.
- Remote capture arrives over ZMQ SUB in `MasterCoordinator._data_receiver()`.
- **Drain behavior (updated March 2026):** All buffered messages are received and processed per poll cycle (CONFLATE=0). Prior to this fix, the loop drained to latest-only, losing frames needed for sync matching.
- Each buffered frame: `{camera_id, frame_number, timestamp_ns, results, received_at}`.
- `frame_buffers` for `local_cam` and `cam_0` are **pre-registered at init** — the sync gate no longer stalls waiting for the first remote frame to create the buffer entry.

### Stage 2 — Clock Correction & Sync Gate

- Clock correction: `frame_data.timestamp += clock_offsets[camera_id]` (nanoseconds), applied per-frame using Cristian's Algorithm median offset.
- `get_synchronized_batch()` (updated March 2026):
  - **Nearest-frame matching**: anchor = slowest camera's newest frame; each other camera contributes its frame with minimum `|delta|` from anchor. Replaces original oldest-frame anchor.
  - Accepts pair if every camera's best frame is within `SYNC_TIME_THRESHOLD_MS = 200ms`.
  - Stale eviction: remote buffer cleared if newest frame is older than `STALE_FRAME_TIMEOUT_MS = 5000ms` relative to global newest.
- Drop gate in `_process_frame_data()` uses explicit `is None` checks — `timestamp=0` and empty dicts are no longer dropped.
- Network latency logged (throttled) when corrected delay > 300ms.

### Stage 3 — Matched Frame Pair

- Both `local_cam` and `cam_0` frames matched; consumed from deques.
- Each `FrameData.results` holds the output of `compact_results_for_sync()`: `{pose_landmarks, pose_world_landmarks, packet_landmarks, gpu_compute}`. Raw MediaPipe objects and all face/hand data are stripped before buffering.

### Stage 4 — Triangulate

Executed inside `get_synced_3d_pose()`:
- 2D landmark source per remote frame: `_extract_pose_landmarks()` checks `packet_landmarks` first (`{x,y,conf}` compact), then falls back to `results['pose']` MediaPipe object (local camera) or serialized `pose_landmarks` list.
- Normalized (0–1) coordinates are scaled to pixels using the camera's `image_size` from calibration before passing to DLT. Default fallback: 1280×720.
- Confidence gate: `STEREO_POINT_MIN_INPUT_CONFIDENCE = 0.5` — landmarks below threshold are excluded per view.
- Tiered reconstruction per landmark:
  1. **Tier 1:** Multi-view DLT triangulation via `Triangulator.triangulate_point()` when ≥2 views pass the confidence gate. Returns `None` if `reproj_error > REPROJECTION_ERROR_THRESHOLD = 15.0 px`.
  2. **Tier 2:** Monocular back-projection fallback (`_get_monocular_fallback`) when ≤1 view available.
  3. **Tier 3:** Averaged MediaPipe `pose_world_landmarks` (hip-relative, metric) when both Tier 1 and 2 fail.
- Occlusion state machine holds/predicts positions for up to `OCCLUSION_MAX_PREDICTED_FRAMES = 15` hidden frames.

### Stage 5 — 3D Frame Object

Output keys: `pose_3d`, `kinematics_3d`, `low_reliability_landmarks`, `timestamp_ns`, `uncertainty`, `occlusion_states`, `fusion`, `uncertainty_detailed`, `uncertainty_summary`, `advanced_kinematics`, `dashboard_status`.

Pre-packaging: world-axis transform applied; optional 3D One-Euro filter on `x/y/z`.

### Stage 6 — Kinematics

- `KinematicsEngine.process_frame()`: `joint_velocity_3d`, `joint_acceleration_3d`, `joint_angles_deg`, `angular_velocity_deg_s`, `spine_vector`, `flat_export`.
- `AdvancedKinematics.process_frame()`: exported under `advanced_kinematics`.

### Stage 7 — Store

- **Stereo sync path** (`len(synced_batch) >= 2`): `db.save_synced_frame(ts, pc1_res, pc2_res, pose_3d)` — both cameras logged.
- **Fallback path** (no sync / single camera): `db.save_synced_frame(ts, pc1_res, windows_res, mono_pose_3d)` where `windows_res` is taken from `coordinator.frame_buffers['cam_0'][-1]` if available. Windows data is logged even when strict timestamp sync fails.
- DB write queued → `database._worker_loop()` batch inserts (up to 50 per commit).
- `_master_result_queue` maxsize = 20 to prevent overwrite before main thread drains.
- **`pose_world_landmarks` are not stored in the DB** — they exist only as a Tier-3 triangulation fallback in memory.

### Stage 8 — Display

- Side-by-side dual view: local + latest remote JPEG.
- Remote metrics updated from buffer even without sync.
- Live 3D visualizer (`live_viz`) throttled to ~2fps.

---

## 6) Active Multi-Camera Configuration (from `config.py`)

**Mac (`Motion-capture/config.py`):**
- `NUM_CAMERAS = 2`
- `DISCOVERY_PORT = 6000`
- `DATA_PORT = 6001`
- `FEEDBACK_PORT = 6002`
- `CLOCK_SYNC_PORT = 6003`
- `SYNC_TIME_THRESHOLD_MS = 200.0` ← relaxed from 50ms (March 2026)
- `SYNC_DYNAMIC_THRESHOLD_ENABLED = False` ← disabled; was shrinking window to ~16ms at 30fps
- `FRAME_BUFFER_SIZE = 30` ← expanded from 2 (March 2026)
- `STALE_FRAME_TIMEOUT_MS = 5000`
- `TRIANGULATION_MIN_VIEWS = 2`
- `REPROJECTION_ERROR_THRESHOLD = 15.0`
- `STEREO_POINT_MIN_INPUT_CONFIDENCE = 0.5`
- `KINEMATICS_MIN_POINT_CONFIDENCE = 0.5`
- `ENABLE_3D_ONE_EURO_FILTER = True`
- `OCCLUSION_FILL_ENABLED = True`

**Windows (`Motion-capture-vs1/config.py`):**
- `NETWORK_CAMERA_ID = 'cam_0'`
- `MASTER_IP = '10.137.227.228'`
- `DATA_PORT = 6001`, `DISCOVERY_PORT = 6000`, `CLOCK_SYNC_PORT = 6003`
- `NETWORK_PROTOCOL = 'tcp'`
- `NUM_POSES = 1`, `NUM_FACES = 1`, `NUM_HANDS = 2`
- `MAX_FRAME_WIDTH = 640`
- `NETWORK_STREAM_WIDTH = 640`, `NETWORK_STREAM_HEIGHT = 360`
- `NETWORK_JPEG_QUALITY = 35`
- `ENABLE_CLOCK_SYNC = True`

---

## 7) Windows Config Discrepancies (GitHub vs Required)

The GitHub repo at `github.com/Mrudula-itsjuzme/Motion-capture` contains an older config.
The following settings on that repo differ from what the Windows node needs:

| Setting | GitHub (outdated) | Required (vs1) | Impact |
|---|---|---|---|
| `MASTER_IP` | `192.168.1.100` | `10.137.227.228` | **CRITICAL — Windows cannot connect** |
| `DISCOVERY_PORT` | `5000` | `6000` | **CRITICAL — wrong port** |
| `DATA_PORT` | `5001` | `6001` | **CRITICAL — wrong port** |
| `NETWORK_PROTOCOL` | `'udp'` | `'tcp'` | **CRITICAL — protocol mismatch** |
| `NUM_POSES / NUM_FACES / NUM_HANDS` | `5 / 5 / 5` | `1 / 1 / 2` | High memory usage on Windows |
| `MAX_FRAME_WIDTH` | `1280` | `640` | ~4× extra inference cost |
| `SYNC_TIME_THRESHOLD_MS` | `33.0` | `200.0` | Nearly zero sync success |
| `FRAME_BUFFER_SIZE` | `10` | `30` | Insufficient sync history |
| `NETWORK_CAMERA_ID` | missing | `'cam_0'` | Identity not set |
| `ENABLE_CLOCK_SYNC` | missing | `True` | Clocks not aligned |
| `CLOCK_SYNC_PORT` | missing | `6003` | Clock sync unavailable |
| `FEEDBACK_PORT` | missing | `6002` | No quality feedback |
| `NETWORK_JPEG_QUALITY` | missing | `35` | Full-quality JPEGs sent |
| `NETWORK_STREAM_WIDTH/HEIGHT` | missing | `640 / 360` | Full-res frames sent |
| `STALE_FRAME_TIMEOUT_MS` | missing | `5000` | Unknown eviction behavior |

**Action required: pull the latest `Motion-capture-vs1/config.py` to the Windows node.**

---

## 8) Important Runtime Notes (Current)

- Discovery is manual: `discover_cameras_manual([remote_ip])` called from `launch_multi_camera.py`.
- Local frame ingestion into coordinator occurs in GUI video loop, not over network.
- Frame buffers for both cameras are pre-registered at init — sync gate does not block on first remote frame.
- Sync and triangulation happen only when both camera streams provide matchable timestamps.
- Fallback DB writes now include the latest Windows buffer frame when strict sync fails.
- ArUco stereo calibration wizard is fully wired in the GUI (`_launch_calibration_wizard`) and writes `calibration.json` with `local_cam` / `cam_0` IDs. Hot-reload via `coordinator.reload_calibration()` after save.



