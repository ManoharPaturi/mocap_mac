# Motion Capture System Architecture (Implementation-Accurate)

**Version:** VS7.1 (11 March 2026 — buffering fix, async DB worker, evaluation pipeline, packet trim)
**Last Updated:** 11 March 2026
**Scope:** Describes behavior implemented in this repository (`Motion-capture`) for current master/server operation.

---

## 1) System Intent And Design Boundaries

This project is a real-time, two-camera, networked motion capture system designed to:
- ingest local and remote camera streams,
- synchronize frames over non-deterministic networks,
- reconstruct 3D joints with tiered fallback logic,
- compute kinematic outputs,
- persist runtime + analysis data,
- generate evaluation artifacts suitable for research reporting.

Key operational constraints:
- Runtime must remain responsive under bursty Wi-Fi jitter.
- Latest-frame behavior is preferred over perfect frame completeness.
- The system must degrade gracefully when stereo triangulation is not possible.
- Evaluation logging must never crash the capture/compute path.

---

## 2) Runtime Modes

Configured in `config.py` via `MULTI_CAMERA_MODE`:
- `single`: local camera only
- `server`: local camera + network broadcast
- `master`: local camera + remote receive + synchronization + 3D fusion

This document focuses on `master` mode, where the full architecture executes.

---

## 3) Node Topology And Ports

### Windows Server Node
- Captures camera frames
- Runs MediaPipe inference
- Publishes compacted frame payloads over ZMQ
- Replies to clock sync pings

### Mac Master Node
- Captures local frames
- Receives remote frames over ZMQ SUB
- Aligns clocks and forms synchronized frame batches
- Triangulates and computes kinematics
- Persists to SQLite
- Produces evaluation and reporting outputs

### Active Port Map (from `config.py`)
- `DISCOVERY_PORT=6000`
- `DATA_PORT=6001`
- `FEEDBACK_PORT=6002`
- `CLOCK_SYNC_PORT=6003`

---

## 4) End-To-End Dataflow

```text
Windows camera capture
  -> CameraServer.send_frame_data()
  -> ZMQ PUB (latest-only)
  -> MasterCoordinator._data_receiver() thread
  -> MasterCoordinator._process_frame_data()
  -> frame_buffers['cam_0']

Mac local capture
  -> GUI video loop
  -> frame_buffers['local_cam']

Sync + compute
  -> get_synchronized_batch()
  -> _master_compute_worker thread
  -> get_synced_3d_pose()
  -> kinematics engines
  -> _master_result_queue

Persistence + reporting
  -> _drain_master_results() on main thread
  -> database.save_synced_frame(...)
  -> SQLite worker writes
  -> evaluator.record_frame(...)
  -> results/<session>/evaluation/*
```

---

## 5) Concurrency And Ownership Model

### Threads / Loops
- GUI main thread:
  - local capture, UI rendering, queue draining, DB enqueue
- `_data_receiver` background thread:
  - non-blocking ZMQ SUB receive, deserialization, ingest
- `_master_compute_worker` background thread:
  - sync selection, triangulation, kinematics, evaluation source prep
- `_db_save_worker` background thread (added 11 March 2026):
  - drains async `_db_save_queue`; saves synced frames and mono fallback frames without blocking the capture loop
- DB worker thread (`database._worker_loop`):
  - batched SQLite commits

### Critical Queues / Buffers
- `frame_buffers[cam_id]`: bounded `deque` for local + remote temporal windows
- `_master_result_queue`: bounded queue (`maxsize=2`) for compute->main handoff (reduced from 20 to prevent stale-result backlog)
- `_db_save_queue`: unbounded queue drained by `_db_save_worker` thread — save tasks are enqueued from the capture loop and committed asynchronously, eliminating SQLite stalls from the hot path
- DB write queue: producer/consumer buffer for insert batching

### Why this split
- Protect UI responsiveness from compute and I/O spikes.
- Keep network receive independent from triangulation throughput.
- Decouple DB latency from compute path.

---

## 6) Network Transport Contract

### ZMQ Strategy
- Publisher/subscriber configured for latest-only semantics:
  - sender side high-water mark `SNDHWM=1` (single-frame send queue),
  - receiver side high-water mark low,
  - `CONFLATE=1` on PUB socket — only the latest frame survives the buffer.

This intentionally trades completeness for timeliness: stale frames are dropped immediately so the receiver always processes the camera-live frame, not a buffered one.

### Camera Buffer Strategy
- `CAP_BUFFERSIZE=1` on `VideoCapture` flushes stale driver frames before each read.
- Windows path uses `CAP_DSHOW` backend for reliable buffer-size enforcement.
- Mac path uses default AVFoundation backend.

### Remote Packet Shape (logical)
- identity: `camera_id`, `frame_number`, `schema_version`, `calibration_id`
- timing:
  - `timestamp`: actual send time (nanoseconds at point of ZMQ publish)
  - `capture_timestamp_ns`: frame-grab time (diagnostic; may differ from send time by processing delay)
- payload:
  - compact 2D landmarks (`landmarks` field — list of 33 `{x, y, conf}` dicts)
  - `pose_world_landmarks` (Tier-3 fallback; hip-relative; small)
  - compressed JPEG bytes (remote display only; not used in compute path)

> Face and hand landmark payloads are **not transmitted** (stripped at source since 11 March 2026 to reduce per-packet size).

### Ingest Rules
- Non-conforming payloads are rejected safely.
- Remote frames with extreme corrected latency are dropped by ingress guard.
- JPEG and landmarks are separated; JPEG does not pollute compute data structures.

---

## 7) Time Sync And Frame Synchronization

### Clock Sync
- Master periodically estimates per-camera offsets via ping/pong exchange.
- Corrected timestamps are used for downstream sync.
- If samples are weak, passive offset fallback is used.

### Sync Gate
`get_synchronized_batch()` forms per-frame batches using nearest-neighbor timestamp matching:
- anchor: newest frame from slowest camera stream,
- each other stream contributes closest timestamp frame,
- accepted if all deltas are within `SYNC_TIME_THRESHOLD_MS`.

Current threshold from `config.py`:
- `SYNC_TIME_THRESHOLD_MS=300.0`

Buffer settings from `config.py`:
- `FRAME_BUFFER_SIZE=4`
- `STALE_FRAME_TIMEOUT_MS=2500`

Behavioral effect:
- higher threshold increases batch success under network jitter,
- lower buffer size reduces RAM but narrows matching window.

---

## 8) 3D Reconstruction Pipeline

Entry point: `MasterCoordinator.get_synced_3d_pose()`

### Landmark Source Priority
For each camera frame, extractor checks compact payload first, then fallbacks.

### Per-Joint Tiered Reconstruction
1. Tier 1: stereo triangulation (DLT), requires minimum view/confidence gates.
2. Tier 2: monocular fallback when stereo is insufficient.
3. Tier 3: pose-world fallback when both above fail.

Config-driven gates:
- `TRIANGULATION_MIN_VIEWS=2`
- `STEREO_POINT_MIN_INPUT_CONFIDENCE=0.5`
- `REPROJECTION_ERROR_THRESHOLD=15.0`

### Occlusion Handling
Occlusion state machine keeps continuity for temporarily invisible points using bounded prediction windows.

### Post-Reconstruction Filtering
- Optional 3D One-Euro filtering before kinematics:
  - `ENABLE_3D_ONE_EURO_FILTER=True`

---

## 9) Kinematics Layer

Two processing blocks run on the reconstructed 3D skeleton:
- baseline kinematics engine:
  - linear velocities/accelerations,
  - joint angles,
  - angular rates,
  - flattened export map.
- advanced kinematics engine:
  - additional derived outputs for analytics/dashboarding.

Confidence floor for kinematic use:
- `KINEMATICS_MIN_POINT_CONFIDENCE=0.5`

---

## 10) Persistence Model

Primary persistence API:
- `database.save_synced_frame(timestamp, pc1_results, pc2_results, pose_3d)`

Storage behavior:
- synchronized stereo path stores both camera payloads and fused 3D,
- fallback path still stores latest available remote payload + mono/fallback 3D,
- DB worker commits in batches to reduce write overhead.

Session partitioning:
- recording creates a session table `session_YYYYMMDD_HHMMSS`.
- per-session reports are generated in `results/<session>/`.

---

## 11) Evaluation And Reporting Architecture

Runtime evaluator class:
- `src/evaluation_pipeline.py:MotionCaptureEvaluationPipeline`

Coordinator integration points:
- constructed during coordinator init,
- bound to database context,
- receives calibration and optional GT path,
- called per frame from compute path,
- finalized on coordinator shutdown.

### Per-Frame Outputs
`results/<session>/evaluation/per_frame_metrics.csv` columns include:
- `reconstruction_accuracy`
- `mpjpe_m`
- `reprojection_error_px`
- `epipolar_error_px`
- `bone_length_variance_m`
- `joint_jitter_m`
- `occlusion_recovery_ms`
- `calibration_rms_px`
- `pipeline_latency_ms`
- `fps`
- `network_latency_ms`
- `packet_loss_rate`
- `synchronization_error_ms`
- `triangulation_success_rate`
- `depth_stability_m`
- `cpu_percent`
- `memory_mb`
- point counts (`triangulated_points`, `total_points`)

### Aggregate Outputs
- `aggregate_metrics.json` (mean/std/p95/min/max)
- `evaluation_table.md` (publication-ready markdown)
- plot set:
  - `latency_kde.png`
  - `bone_variance_line.png`
  - `jitter_scatter.png`

### Stable Latest Pointer
- `results/latest_evaluation_artifacts.json`
- points tooling to most recent evaluation artifacts.

---

## 12) Evaluation Robustness Fixes (Current)

Recent stability updates in evaluator:
- restored resource usage method to always return `(cpu_percent, memory_mb)` safely,
- metric-level guard wrapper (`_safe_metric`) prevents single metric failure from dropping entire frame row,
- FPS estimation hardened:
  - now prefers timestamp delta between synchronized frames,
  - rejects unrealistic instantaneous rates,
  - bounded fallback to latency-derived estimate,
- session label stabilization:
  - once `session_*` recording label is active, evaluator keeps that label,
  - avoids later `live_*` pointer overwrites after recording stop.

Operational impact:
- no more `cannot unpack non-iterable NoneType` evaluator crashes,
- per-frame CSV reliably populated,
- FPS metric no longer explodes due to invalid micro-latency source.

---

## 13) Failure Modes And Degradation Paths

### Remote lag / jitter spikes
- effect: fewer synchronized batches, lower triangulation quality.
- mitigation: nearest-frame matching, relaxed sync threshold, stale eviction, latest-only transport.

### Low confidence landmarks
- effect: fewer joints pass Tier 1 triangulation.
- mitigation: Tier 2 monocular and Tier 3 pose-world fallback.

### Missing calibration file
- effect: baseline default calibration used; geometric metrics are less trustworthy.
- mitigation: run stereo calibration wizard and reload.

### DB backpressure
- effect: delayed commits under extreme load.
- mitigation: async DB worker and batching.

### Evaluator exceptions
- effect before fix: silent row loss or repeated runtime errors.
- mitigation now: safe metric wrappers, explicit coordinator-side error logs, robust method returns.

---

## 14) Performance And Resource Characteristics

Observed in latest runs:
- pipeline latency in practical real-time range (~tens of ms mean),
- memory footprint generally a few hundred MB during active capture,
- CPU can exceed 100% on multi-core systems (process-level metric).

Important interpretation note:
- process `cpu_percent` from `psutil` is not capped at 100 on multi-core machines.

---

## 15) Verification Runbook

After a recording session:
1. Confirm CSV rows exist (not header-only).
2. Confirm aggregate JSON and plots exist.
3. Confirm `results/latest_evaluation_artifacts.json` points to intended session.
4. Review key metrics:
   - `pipeline_latency_ms`
   - `fps`
   - `triangulation_success_rate`
   - `reprojection_error_px`
   - `epipolar_error_px`

Suggested shell checks:
```bash
wc -l results/session_*/evaluation/per_frame_metrics.csv | tail -1
ls -lah results/session_*/evaluation | tail -10
cat results/latest_evaluation_artifacts.json
```

---

## 16) Source Map (Where To Read In Code)

Core files:
- `launch_multi_camera.py` - startup and mode wiring
- `main_gui.py` - UI loop, local ingestion, display
- `src/master_coordinator.py` - network receive, sync, reconstruction orchestration, evaluation calls
- `src/triangulation.py` - geometric reconstruction
- `src/database.py` - async persistence
- `src/evaluation_pipeline.py` - metrics, aggregates, plots, artifact index
- `config.py` - active runtime constants and thresholds

---

## 17) Current Gaps / Next Improvements

- Re-run full capture after FPS patch to refresh aggregate FPS in published results.
- Improve stereo calibration availability and persistence checks to reduce epipolar/reprojection drift.
- Add automated checks that fail CI when evaluation CSV has header-only output.
- Add an explicit metric-quality flag in aggregate output (for example, `fps_valid=true/false`).
- Optional: restore `LiveVisualizer3D` as an on-demand side-window (currently removed from GUI hot path).
- Harden `_db_save_worker` with explicit flush-on-shutdown (currently worker exits on `running=False`).
