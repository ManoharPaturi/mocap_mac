# Motion Capture — Changes Log

**Last Updated**: 11 March 2026  
**Repos**: `Motion-capture/` (Mac master) · `Motion-capture-vs1/` (Windows server)

---

## Session 4 — 10–11 March 2026 (Buffering, Async DB, Evaluation Pipeline)

### Fix 1 — Live video feed buffer buildup eliminated

**Root cause**: `SNDHWM=5` and `CONFLATE=0` on the ZMQ PUB socket allowed a
queue of up to 5 stale frames to accumulate on the sender side. The master
consumed these buffered frames before seeing the live frame, introducing
visible lag (1–5 second delay on a Wi-Fi link).

**Fix**:
- `camera_server.py` — `SNDHWM` reduced from `5` → `1`; `CONFLATE` enabled (`1`).
  Only the latest frame survives in the send buffer; stale frames are dropped.
- `src/camera.py` — `CAP_BUFFERSIZE=1` set on `VideoCapture` to flush stale
  camera-driver frames before each read.
- Windows path: `CAP_DSHOW` backend forced for reliable buffer-size support.
- Master receiver: waits for a fresh frame (`grab()` + `retrieve()` pattern) so
  the processing loop never re-uses the previous frame's pixel data.

---

### Fix 2 — Asynchronous DB writes decoupled from capture loop

**Problem**: `db.save_synced_frame()` was called inline in the main video thread.
SQLite batch commits (every 50 frames) caused 20–40 ms stalls, manifesting as
dropped frames and uneven capture timing.

**Fix**:
- `main_gui.py` — added `_db_save_queue` (`queue.Queue()`, unbounded) and a
  dedicated `_db_save_worker` daemon thread.
  The capture loop enqueues save tasks (`('synced', ...)` or `('mono', ...)`)
  and returns immediately. The worker thread drains saves in the background.
- A save-path tracing counter (`_save_trace`) records enqueue/dequeue counts
  every 30 frames to aid diagnosis of lost PC2 or 3D frame data.

---

### Fix 3 — `_master_result_queue` size reduced (20 → 2)

**Problem**: With `maxsize=20`, the result queue could hold 20 stale compute
results from before a calibration change or connection drop. Downstream display
code rendered outdated poses until the queue drained.

**Fix**:
- `main_gui.py` — `_master_result_queue = queue.Queue(maxsize=2)`.
  The UI always reflects ≤2-frame-old compute output.

---

### Fix 4 — Face / hand / 2D pose landmarks stripped from network packet

**Problem**: Each ZMQ frame packet included full serialized `pose_landmarks`,
`face_landmarks`, and `hand_landmarks` objects (≈8–25 KB extra per frame).
The master coordinator never consumed face or hand data from the remote; sending
them wasted bandwidth and inflated per-packet latency.

**Fix**:
- `camera_server.py` — `send_frame_data()` now serializes only:
  - compact 2D landmarks (`landmarks` field via `_build_stereo_packet_landmarks`)
  - `pose_world_landmarks` (Tier-3 fallback, small)
  - JPEG frame bytes
  Face and hand landmark blobs are no longer serialized or transmitted.
- `send_timestamp_ns` (actual send time) is now separate from
  `capture_timestamp_ns` (frame grab time), so sender processing delay no longer
  masquerades as network latency on the master.

---

### Fix 5 — fps_label.config moved off video thread to prevent Tk stalls

**Problem**: Calling `fps_label.config(text=...)` directly inside the OpenCV
video thread triggered Tcl/Tk cross-thread access. On Windows this caused
intermittent freeze or crash.

**Fix**:
- `main_gui.py` — FPS string is written to a thread-safe variable; main thread
  applies `.config()` during the scheduled `_poll()` update.

---

### Fix 6 — Passive clock offset fallback for weak sync samples

**Problem**: When the Windows firewall was partially blocking port 6003, the
clock-sync ping/pong often yielded < `CLOCK_SYNC_MIN_VALID_SAMPLES` valid
measurements. Coordinator fell back to zero offset, corrupting sync matching.

**Fix**:
- `src/master_coordinator.py` — stores a `_passive_clock_offset_ns` derived
  from first-packet timestamp comparison. If active clock sync yields fewer than
  `CLOCK_SYNC_MIN_VALID_SAMPLES` samples, passive offset is used instead of
  zero.
- `_clock_sync_failures` counter added; logged on each sync attempt.

---

### Fix 7 — Per-camera frame rate tracking + adaptive sync threshold bounds

**Problem**: `SYNC_DYNAMIC_THRESHOLD_ENABLED` widened the sync window based on
signal variance, but had no floor or ceiling. Extreme jitter could push the
threshold to several seconds, accepting wildly mismatched frames.

**Fix**:
- `src/master_coordinator.py` — added `camera_frame_rates`, `camera_interval_samples`,
  and `last_buffer_sizes` tracking per camera.
- Adaptive threshold now clamped between `SYNC_THRESHOLD_MIN_MS` and
  `SYNC_THRESHOLD_MAX_MS` (new `config.py` params, defaults: 15 ms / 500 ms).

---

### Fix 8 — `LiveVisualizer3D` and `Visualizer3D` removed from GUI hot path

**Problem**: `LiveVisualizer3D` (Matplotlib window) was initialized at startup
even when not in use, holding a ~80 MB GPU/RAM footprint. `Visualizer3D` kept
an open Plotly HTML file handle.

**Fix**:
- `main_gui.py` — both imports and instance creation removed.
  `show_visualization()` now calls `ReportGenerator` only (static PNG report).
- "Current Live 3D" button removed from control panel.
- Net RAM saving: ~80–120 MB on master node.

---

### New — `src/evaluation_pipeline.py` (runtime evaluation metrics)

Added `MotionCaptureEvaluationPipeline`:
- Computes per-synchronized-frame metrics: `mpjpe_m`, `reprojection_error_px`,
  `epipolar_error_px`, `bone_length_variance_m`, `joint_jitter_m`,
  `occlusion_recovery_ms`, `pipeline_latency_ms`, `triangulation_success_rate`,
  `network_latency_ms`, `synchronization_error_ms`, `packet_loss_rate`,
  `cpu_percent`, `memory_mb`, `depth_stability_m`.
- Writes `results/<session>/evaluation/per_frame_metrics.csv` and
  `aggregate_metrics.json` during live sessions.
- Generates figures: `latency_kde.png`, `bone_variance_line.png`, `jitter_scatter.png`.
- Auto-flushes every 60 frames; final flush on `finish_session()`.
- Ground-truth support: load a JSON file (`load_ground_truth()`) for MPJPE
  computation against reference poses.

---

### New — `src/results_report.py` + `results.md` (consolidated report)

Added `write_consolidated_results_report()`:
- Scans `results/session_*/evaluation/aggregate_metrics.json` for the latest
  session, then formats a Markdown table with mean/P95/max for all key metrics.
- Writes `results.md` at repo root. Embeds figure links for
  `latency_kde.png`, `bone_variance_line.png`, `jitter_scatter.png`.
- Wired into `evaluation_pipeline.finish_session()` — runs automatically at
  session end.

---

### New — `tools/generate_results_report.py` (standalone report regeneration)

CLI tool: `python tools/generate_results_report.py [--results-dir results]`
Regenerates `results.md` from whatever session artifacts are present on disk
without needing to run a live capture.

---

### New — `wiretap.py` + `check.py` (diagnostics)

- `wiretap.py`: connects to the Windows data port (ZMQ SUB, port 6001),
  receives a single frame, and dumps all top-level keys with type and size
  metadata. Useful for verifying packet shape across refactors.
- `check.py`: reads `mocap_data.db` and reports the most recent session table's
  PC2 frame count and 3D pose frame count. Quick sanity check that pipeline data
  is reaching SQLite.

---

## Session 3 — 9 March 2026 (Connection + GPU Crash fixes)

### Fix 1 — MediaPipe MPS/Metal fatal crash (exit 134 / SIGABRT)

**Root cause**: `MAX_FRAME_WIDTH = 640` resizes a 720p camera to height `int(720 * 0.5) = 360`.
Apple Silicon CVPixelBuffer requires frame height to be a **multiple of 16**.
`360 mod 16 = 8` → `kCVReturnInvalidSize (-6662)` → hard `abort()` in C++ → Python cannot catch it.

**Files changed**:
- `src/detector.py` (both repos) — both resize call sites now round `new_h` up to nearest 16 when using GPU delegate:
  ```python
  new_h = int(H_orig * scale)
  if self.use_gpu_delegate:
      new_h = ((new_h + 15) // 16) * 16
  ```
- `config.py` (Mac) — `INFERENCE_BACKEND` restored to `'mps'`; `PREFER_GPU_DELEGATE = True`

---

### Fix 2 — Automatic MPS→CPU fallback (subprocess retry)

**Problem**: If MediaPipe GPU crashes for any unforeseen reason, the app dies with no recovery.

**Files changed**:
- `launch_multi_camera.py` (Mac) — rewritten from a `runpy` one-liner to a subprocess launcher.
  Detects exit code `134` (SIGABRT) and retries with `MOCAP_BACKEND_OVERRIDE=cpu`:
  ```
  attempt 1 → MPS (normal)
    └─ exit 134? → attempt 2 → CPU (override)
  ```
- `tools/launch_multi_camera.py` (Mac) — reads `MOCAP_BACKEND_OVERRIDE` env-var at startup
  and forces `config.INFERENCE_BACKEND = 'cpu'` before MediaPipe is imported.

---

### Fix 3 — Windows firewall blocking all app traffic

**Root cause**: `scripts/allow_firewall.ps1` opened ports **5000 and 5001** (wrong).
The app actually uses:

| Port | Purpose |
|------|---------|
| 6000 | `DISCOVERY_PORT` |
| 6001 | `DATA_PORT` |
| 6002 | `FEEDBACK_PORT` |
| 6003 | `CLOCK_SYNC_PORT` |

Windows firewall blocked all data and clock-sync traffic → Windows dropped after ~100 frames,
ClockSync always returned "No valid samples" for port 6003.

**Files changed**:
- `scripts/allow_firewall.ps1` (both repos) — replaced entirely.
  Removes old 5000/5001 rules; adds inbound **and** outbound rules for 6000–6003.
  **Must be re-run as Administrator on the Windows PC.**

---

### Fix 4 — Frame sync threshold widened for raw clock skew

**Problem**: `SYNC_TIME_THRESHOLD_MS = 15.0` — Windows wall clock is ~600 ms behind Mac
before ClockSync corrects it. With ClockSync blocked (Fix 3 not yet applied), zero frames matched.

**Files changed**:
- `config.py` (Mac) — `SYNC_TIME_THRESHOLD_MS = 700.0`; `SYNC_DYNAMIC_THRESHOLD_ENABLED = False`.

> **Revert after Fix 3 is applied on Windows**: once ClockSync logs show offset+RTT,
> set `SYNC_TIME_THRESHOLD_MS = 50.0` and `SYNC_DYNAMIC_THRESHOLD_ENABLED = True`.

---

### Fix 5 — `_get_rss_mb()` returned 0.0 when psutil not installed

**Root cause**: The `except ImportError` branch just `return 0.0`.

**Files changed**:
- `main_gui.py` (Mac) — fallback now uses `resource.getrusage` (stdlib, no install needed).
  Added `_RSS_IS_PEAK` flag. On macOS `ru_maxrss` is a high-water mark (never decreases),
  so logs say `(peak/maxrss — install psutil for current)`.

---

### Fix 6 — DISCONNECTED display state for dropped remote camera

**Problem**: When Windows stopped sending frames, the remote panel showed `WAITING` forever.

**Files changed**:
- `main_gui.py` (Mac) — added `self._last_remote_frame_time` tracking.
  After 5 s of silence, `sync_label` changes to `"DISCONNECTED"` and label turns **red**.

---

## Session 2 — 4–7 March 2026 (RAM crash + feed freeze)

### RAM crash — 7 root causes fixed

| # | Root Cause | Fix | File |
|---|-----------|-----|------|
| 1 | `MAX_FRAME_WIDTH = 1280` — 1280x720 Metal inference used 200–400 MB/frame | Changed to `640` | `config.py` |
| 2 | `ImageTk.PhotoImage` created fresh every frame — never GC'd by Tk | Reuse via `.paste()` on single persistent `PhotoImage` | `main_gui.py` |
| 3 | `np.hstack([local, remote])` at 640x480x3 kept alive every frame | Throttle to every 3rd frame (`% 3`); `del combined` immediately; resize to 640x360 | `main_gui.py` |
| 4 | CLAHE intermediates (`lab, l, a, b, cl, limg`) never freed | `del` after use | `src/detector.py` |
| 5 | `mp_image` source arrays kept alive inside Metal buffer | `del frame_rgb` after `mp.Image()` | `src/detector.py` |
| 6 | Duplicate `draw_landmarks()` — rendering landmarks twice per frame | Removed duplicate | `src/visualizer.py` (vs1) |
| 7 | Python GC never invoked | `gc.collect()` every 150 frames | `main_gui.py` |

### Feed freeze fix

**Root cause**: `_poll()` (Tkinter main-thread poller) had no `try/finally`. Any exception
permanently stopped rescheduling → display and metrics froze forever.

**Fix**: Wrapped body in `try/finally`; `finally` always reschedules `self.root.after(15, _poll)`.
**File**: `main_gui.py`

### Other Session 2 changes

- `FrameData` import hoisted to top of file (was imported inside loop 30x/s)
- Display resolution corrected to `640x360` (16:9) from `320x240`
- `psutil` added to `requirements.txt`
- Ablation study flags added to `config.py`:
  `ABLATION_DISABLE_FRAME_RESIZE`, `ABLATION_DISABLE_DISPLAY_THROTTLE`,
  `ABLATION_DISABLE_PHOTO_REUSE`, `ABLATION_DISABLE_CLAHE_DEL`, `ABLATION_RSS_LOG_INTERVAL = 60`
- `[ABLATION]` RSS telemetry logged every 60 frames
- All Session 2 fixes mirrored to `Motion-capture-vs1/`

---

## Session 1 — March 2026 (Architecture)

- 8-stage pipeline: Capture → Sync → Match → Triangulate → 3D Object → Kinematics → Store → Display
- Dynamic sync threshold (`SYNC_DYNAMIC_THRESHOLD_ENABLED`, `SYNC_DYNAMIC_FACTOR`)
- Clock sync thread (`_clock_sync_loop`) with RTT-based offset estimation
- Occlusion fusion with velocity-based joint prediction for hidden joints
- `processing_metadata` table in SQLite for self-aware session context
- ArUco calibration flow (intrinsic + stereo APIs, board generator)
- 4-layer database schema (RAW → RECONSTRUCTION → DERIVED → VALIDATION)
- Cross-camera skeleton overlay (remote landmarks on local frame for occluded joints)
- Pipelined master compute worker — capture thread decoupled from sync/3D compute
- Bounded queues: DB write (`maxsize=500`), UI task (`maxsize=256`), network send (`maxsize=1`)
- Latency, reconstruction quality, and clock-sync panels in master dashboard

---

## Windows PC — Required actions (do once)

1. **Re-run the firewall script** (replaces old wrong 5000/5001 rules with 6000–6003):
   ```powershell
   # PowerShell as Administrator
   cd path\to\Motion-capture-vs1\scripts
   .\allow_firewall.ps1
   ```
   Verify: `netstat -an | findstr "600"` — ports 6000–6003 should appear.

2. **Start the server**:
   ```
   python launch_multi_camera.py --mode server
   ```

3. Once ClockSync logs show offset + RTT (not "No valid samples"), tighten sync on Mac:
   ```python
   # config.py on Mac
   SYNC_TIME_THRESHOLD_MS = 50.0
   SYNC_DYNAMIC_THRESHOLD_ENABLED = True
   ```
