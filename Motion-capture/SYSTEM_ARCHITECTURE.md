# Motion Capture System Architecture (Implementation-Accurate)

**Version:** VS5 (March 2026 — post pipeline-reliability update)
**Last Updated:** 9 March 2026  
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
      │   clock-offset correction → frame_buffers['cam_0'] (deque maxlen=30)
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

## 4) Stage Behavior (Exactly What Code Executes)

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
- Compact results: `{pose_landmarks, pose_world_landmarks, packet_landmarks}` — raw MediaPipe objects, face, and hand data stripped from network frames.

### Stage 4 — Triangulate

Executed inside `get_synced_3d_pose()`:
- Tiered per-landmark reconstruction:
  1. **Tier 1:** DLT triangulation when ≥2 views with confidence ≥ `STEREO_POINT_MIN_INPUT_CONFIDENCE`.
  2. **Tier 2:** Monocular back-projection fallback (`_get_monocular_fallback`).
  3. **Tier 3:** Averaged MediaPipe world-landmarks fallback.
- Occlusion state machine holds/predicts landmarks for up to 15 hidden frames.

### Stage 5 — 3D Frame Object

Output keys: `pose_3d`, `kinematics_3d`, `low_reliability_landmarks`, `timestamp_ns`, `uncertainty`, `occlusion_states`, `fusion`, `uncertainty_detailed`, `uncertainty_summary`, `advanced_kinematics`, `dashboard_status`.

Pre-packaging: world-axis transform applied; optional 3D One-Euro filter on `x/y/z`.

### Stage 6 — Kinematics

- `KinematicsEngine.process_frame()`: `joint_velocity_3d`, `joint_acceleration_3d`, `joint_angles_deg`, `angular_velocity_deg_s`, `spine_vector`, `flat_export`.
- `AdvancedKinematics.process_frame()`: exported under `advanced_kinematics`.

### Stage 7 — Store (Updated March 2026)

- **Stereo sync path** (`len(synced_batch) >= 2`): `db.save_synced_frame(ts, pc1_res, pc2_res, pose_3d)` — both cameras logged.
- **Fallback path** (no sync / single camera): `db.save_synced_frame(ts, pc1_res, windows_res, mono_pose_3d)` where `windows_res` is taken from `coordinator.frame_buffers['cam_0'][-1]` if available. **Windows data is now logged even when strict timestamp sync fails.**
- DB write queued → `_worker_loop()` batch inserts (up to 50 per commit).
- `_master_result_queue` maxsize raised to 20 (was 2) to prevent overwrite before main thread drains.

### Stage 8 — Display

- Side-by-side dual view: local + latest remote JPEG.
- Remote metrics updated from buffer even without sync.
- Live 3D visualizer (`live_viz`) throttled to ~2fps.

---

## 5) Active Multi-Camera Configuration (from `config.py`)

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

## 6) Windows Config Discrepancies (GitHub vs Required)

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

## 7) Important Runtime Notes (Current)

- Discovery is manual: `discover_cameras_manual([remote_ip])` called from `launch_multi_camera.py`.
- Local frame ingestion into coordinator occurs in GUI video loop, not over network.
- Frame buffers for both cameras are pre-registered at init — sync gate does not block on first remote frame.
- Sync and triangulation happen only when both camera streams provide matchable timestamps.
- Fallback DB writes now include the latest Windows buffer frame when strict sync fails.
- ArUco stereo calibration wizard is fully wired in the GUI (`_launch_calibration_wizard`) and writes `calibration.json` with `local_cam` / `cam_0` IDs. Hot-reload via `coordinator.reload_calibration()` after save.


---

## 1) Runtime Modes

Configured in `config.py` with `MULTI_CAMERA_MODE`:
- `single`: local camera pipeline only
- `server`: local camera pipeline + network broadcast of frame payloads
- `master`: local camera pipeline + remote receive + synchronization + 3D fusion

This document focuses on `master` mode because that is where multi-camera architecture runs.

---

## 2) Canonical Master Pipeline (Current Runtime)

```
Capture (Front)
Capture (Right)
       │
       ▼
   Time Sync
       │
       ▼
Matched Frame Pair
       │
       ▼
  Triangulate
       │
       ▼
 3D Frame Object
       │
       ▼
  Kinematics
       │
       ▼
    Store
       │
       ▼
   Display
```

Naming used at runtime:
- `Capture (Front)` = local camera frame added as `camera_id='local_cam'`
- `Capture (Right)` = remote camera frame typically received as `camera_id='cam_0'`

---

## 3) Stage Behavior (Exactly What Code Executes)

### Stage 1 — Capture (Front / Right)

- Front/local capture is read in `main_gui.py` video loop and appended to `coordinator.frame_buffers['local_cam']` as `FrameData`.
- Right/remote capture arrives over ZMQ SUB sockets in `MasterCoordinator._data_receiver()` and is parsed in `_process_frame_data()`.
- Each buffered frame carries: `camera_id`, `frame_number`, `timestamp` (epoch ns), `results`, `received_at`.
- If present, `frame_jpeg`, `packet_landmarks`, and `gpu_compute` are attached into `results`.

### Stage 2 — Time Sync

- Optional clock correction is applied per camera in `_process_frame_data()` when `ENABLE_CLOCK_SYNC` is enabled and an offset is known.
- `get_synchronized_batch()` is the sync gate:
  - Requires buffers from all expected cameras (`num_cameras`, default 2).
  - Evicts stale remote buffers using `STALE_FRAME_TIMEOUT_MS`.
  - Matches frames within `SYNC_TIME_THRESHOLD_MS` (converted to ns).

### Stage 3 — Matched Frame Pair

- A matched batch is returned only if one frame per camera satisfies sync threshold.
- Used/older frames are popped from each camera deque after a successful match.
- If any camera has no match, function returns `None` and increments sync failure count.

### Stage 4 — Triangulate

Executed inside `get_synced_3d_pose()`:
- Extracts per-camera 2D landmarks from incoming frame payloads.
- Uses confidence gate `STEREO_POINT_MIN_INPUT_CONFIDENCE` before adding an observation.
- Tiered reconstruction per landmark:
  1. **Tier 1:** Multi-view DLT triangulation via `Triangulator.triangulate_point()` when at least 2 views are available.
  2. **Tier 2:** Monocular back-projection fallback (`_get_monocular_fallback`) when enabled and visibility is sufficient.
  3. **Tier 3:** Average MediaPipe world-landmarks fallback when available.
- Occlusion state machine then applies hold/prediction (`predicted`) up to 15 hidden frames.

### Stage 5 — 3D Frame Object

If reconstruction succeeds, returned object includes these keys:
- `pose_3d`
- `kinematics_3d`
- `low_reliability_landmarks`
- `timestamp_ns`
- `uncertainty`
- `occlusion_states`
- `fusion`
- `uncertainty_detailed`
- `uncertainty_summary`
- `advanced_kinematics`
- `dashboard_status`

Before packaging:
- Axis transform is applied (`WORLD_AXIS_TRANSFORM`).
- Optional 3D One-Euro filter is applied to joint `x/y/z` only (`ENABLE_3D_ONE_EURO_FILTER`).

### Stage 6 — Kinematics

- Base kinematics are produced by `KinematicsEngine.process_frame()` through `_compute_pose_3d_kinematics()`.
- Output includes:
  - `joint_velocity_3d`
  - `joint_acceleration_3d`
  - `joint_angles_deg`
  - `angular_velocity_deg_s`
  - `spine_vector`
  - `flat_export`
- Additional metrics are produced by `AdvancedKinematics.process_frame()` and exported under `advanced_kinematics`.

### Stage 7 — Store

- In master mode recording path, `main_gui.py` calls `db.save_synced_frame(time.time(), pc1_res, pc2_res, pose_3d)`.
- `save_synced_frame()` serializes:
  - PC1 pose/face/hand/derived
  - PC2 pose/face/hand/derived
  - 3D landmarks (`pose_3d` flattened)
  - combined derived block (`kinematics_3d`, reliability, confidence)
- Write is queued and inserted asynchronously in `_worker_loop()` in batches (up to 50 queued items per commit cycle).

### Stage 8 — Display

Master mode display path in `main_gui.py`:
- Builds side-by-side dual view (`local_display` + `remote_display`).
- Remote display uses latest buffered JPEG decode when available.
- Schedules Tkinter display update (`_schedule_display_tkinter`).
- On synced batches, computes 3D pose and updates metrics/quality panels.
- If `live_viz` is initialized, pushes fused 3D frame into live 3D visualizer.

---

## 4) Active Multi-Camera Configuration (from `config.py`)

- `NUM_CAMERAS = 2`
- `DISCOVERY_PORT = 6000`
- `DATA_PORT = 6001`
- `FEEDBACK_PORT = 6002`
- `CLOCK_SYNC_PORT = 6003`
- `SYNC_TIME_THRESHOLD_MS = 100.0`
- `FRAME_BUFFER_SIZE = 2`
- `STALE_FRAME_TIMEOUT_MS = 2000`
- `TRIANGULATION_MIN_VIEWS = 2`
- `REPROJECTION_ERROR_THRESHOLD = 15.0`
- `STEREO_POINT_MIN_INPUT_CONFIDENCE = 0.5`
- `KINEMATICS_MIN_POINT_CONFIDENCE = 0.5`
- `ENABLE_3D_ONE_EURO_FILTER = True`
- `OCCLUSION_FILL_ENABLED = True`

---

## 5) Important Runtime Notes (Current)

- Discovery listener is effectively manual in this build; `discover_cameras_manual([...])` is used for connection setup.
- Local frame ingestion into coordinator occurs in GUI loop (not via network).
- Sync and triangulation happen only when both camera streams provide matchable timestamps.
- Master-mode DB writes are synchronized-frame writes; single-mode writes use `save_frame()`.
