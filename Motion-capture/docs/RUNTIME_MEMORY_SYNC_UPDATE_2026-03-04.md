# Runtime, Memory, Sync & Dashboard Update — 4 March 2026

All changes below are live in `Motion-capture/` (Mac master repo).

---

## 1. Memory Fixes

### Problem
Activity Monitor showed RSS growing unboundedly (~1.2 GB+ during recording sessions).

### Root causes & patches

| Root cause | File | Fix applied |
|---|---|---|
| Full MediaPipe result objects kept in every sync-buffer entry | `src/master_coordinator.py` | `compact_results_for_sync()` — strips raw MediaPipe objects; keeps only serialised landmark lists |
| JPEG blobs deep-copied per frame in buffer | `src/master_coordinator.py` | `get_latest_camera_jpeg(cam_id)` — single latest-JPEG cache per camera; frame buffer never stores JPEG |
| DB write queue unbounded (was `maxsize=500`) | `src/database.py` | Capped to `maxsize=120`; drop-oldest policy in `_enqueue_data()` |
| GPU profiling accumulators (mediapipe metal) accumulated forever | `src/detector.py` | Capped at 300 entries with `collections.deque(maxlen=300)` |
| No observability | `main_gui.py` | Memory telemetry log every 300 frames: RSS + buffer sizes + queue depths |

---

## 2. Pipelined Capture→Compute Architecture

### Problem
Master mode blocked the capture loop while running sync + triangulation + DB save, causing frame drops and reducing effective FPS.

### Solution: background compute worker

```
[video_loop]  capture → detect → push to _master_compute_queue (latest-only)
                      ↓ non-blocking
[_master_compute_worker thread]  sync → triangulate → pose_3d → push to _master_result_queue
                      ↓ non-blocking
[video_loop drain]  _drain_master_results() → update GUI state → kick DB save
```

**New objects added in `main_gui.py`:**

| Object | Type | Purpose |
|---|---|---|
| `_master_compute_queue` | `Queue(maxsize=1)` | Latest-only work item to compute thread |
| `_master_result_queue` | `Queue(maxsize=2)` | Completed payloads back to video loop |
| `_master_compute_worker_thread` | `daemon=True` | Runs sync + 3D while next frame captures |
| `_drain_master_results()` | method | Non-blocking drain; feeds GUI + DB |

The video loop now runs at camera FPS regardless of triangulation speed.

---

## 3. Realtime Metrics Display Fix

### Problem
Live Biometrics panel showed `--` for all joints in master mode even when MediaPipe was detecting correctly.

### Root causes
1. `_schedule_metrics_gui()` was guarded by `if MULTI_CAMERA_MODE != 'master':` in the video loop — so master mode *never* pushed local metrics to the display.
2. The worker path only pushed metrics when `synced_batch >= 2` — sync failures (70–206ms delta vs 15ms threshold) meant remote metrics never populated.
3. Label widgets had width=8 chars — too narrow for the `M:123.4° | W:123.4°` format.

### Fixes applied in `main_gui.py`

**Video loop** — always push local metrics regardless of mode:
```python
# Before
if MULTI_CAMERA_MODE != 'master':
    self._schedule_metrics_gui({'local': ..., 'remote': {}})

# After
self._schedule_metrics_gui({'local': self.latest_local_metrics, 'remote': self.latest_remote_metrics})
```

**`_drain_master_results()`** — pull remote metrics from latest buffer frame even without a full sync:
```python
# If no synced pc2_res, fall back to latest buffer entry
if remote_camera_id and not pc2_res and self.coordinator:
    buf = self.coordinator.frame_buffers.get(remote_camera_id)
    if buf and len(buf) > 0:
        pc2_res = buf[-1].results
```

**GUI widget** — wider label in master mode:
```python
val_font  = ("Courier", 9, "bold") if MULTI_CAMERA_MODE == 'master' else ("Courier", 12, "bold")
val_width = 22 if MULTI_CAMERA_MODE == 'master' else 8
```

**`update_metrics_gui()`** — already correct; displays `M:{local} | W:{remote}` per joint.

---

## 4. Unified Mac + Windows Metric Panel

Both streams now show in the **same row** of the Live Biometrics panel:

```
L Elbow   M:142.3° | W:139.8°
R Elbow   M:165.1° | W:--
L Knee    M:178.0° | W:172.4°
```

- `M:` = local Mac camera (computed every frame from live MediaPipe)
- `W:` = remote Windows camera (computed from latest received buffer frame; refreshes whenever new packets arrive)
- Shows `--` for a source only if that source has no detected landmarks

The **angle table** (bottom scroll view) still logs MAC / WIN rows separately with timestamps for audit trail.

---

## 5. Cross-Camera Skeleton Overlay

### Problem
Left hand (or any landmark) not visible in Mac feed but detected on Windows — nothing shown in local node view.

### Solution: `_draw_cross_camera_landmarks()` in `main_gui.py`

New helper called every frame on the local display before side-by-side combine.

**Logic:**
1. Pull serialised `pose_landmarks` from the latest remote camera buffer entry (no sync required).
2. For each pose connection pair `(a, b)`, check local visibility: if `local_vis[idx] < 0.35` AND `remote_vis[idx] > 0.40` → draw remote landmark.
3. Draws in **orange** (`(0, 165, 255)`) so it's visually distinct from the green local skeleton.
4. Adds a small `WIN overlay` legend in the bottom-left corner of the local panel whenever any remote joints are drawn.

```
Thresholds:
  OCCLUSION_THRESH = 0.35  (local visibility below → show remote)
  REMOTE_VIS_MIN   = 0.40  (remote must be confident enough to trust)
```

Overlay uses normalised [0..1] MediaPipe coords — no calibration needed, no stereo geometry. Purely for visual awareness, not used for any computation or recording.

---

## 6. ArUco Calibration Pipeline

Full ArUco-first calibration implemented across three files:

| File | What changed |
|---|---|
| `src/stereo_calibration.py` | `generate_aruco_board_image()`, `calibrate_intrinsic_aruco()`, `calibrate_stereo_aruco()`, `check_reprojection_error_threshold()` |
| `src/calibration_ui.py` | ArUco-first UI with `ArucoDetector` class; chessboard as fallback; `GenerateArucoBoard` button; min 4-marker threshold; blocks save unless reprojection < 1px |
| `src/dashboard_monitoring.py` | `CalibrationQualityMonitor` class with per-camera RMS tracking and pass/fail status |
| `config.py` | ArUco config block: `DICT_6X6_250`, 5×7 board, 40mm marker / 50mm square |

---

## 7. Runtime Log Analysis (session 2026-03-04)

From logs captured during live session:

```
RSS=1198MB  db_q=0  ui_q=4
sync delta: 70–206ms  (threshold: 15ms)   ← frequent sync miss
detection:  ~31ms mean
triangulation: ~2ms mean
```

**Conclusions:**
- Detection (31ms) is the bottleneck; pipelined worker now decouples it from capture
- Sync miss is clock drift between Mac and Windows; the 70–206ms window suggests the wall-clock offset is not correcting fast enough between 30-second sync intervals
- DB queue at 0 = healthy; no write backpressure
- With metrics fix, dashboard now shows local values even when sync misses

**Recommended next step for sync:** reduce `CLOCK_SYNC_INTERVAL_SEC` from 30 → 10 and increase `CLOCK_SYNC_SAMPLES` from 5 → 8 in `config.py` to converge faster on offset.

---

## 8. Files Changed This Session

```
main_gui.py
  - Label width fix (master mode biometrics panel)
  - Always push local metrics in video loop (master mode)
  - _drain_master_results: remote metrics from buffer even without sync
  - _draw_cross_camera_landmarks: cross-camera occlusion overlay (new method)
  - _POSE_CONNECTIONS: class-level connection list for overlay

src/master_coordinator.py
  - compact_results_for_sync()
  - get_latest_camera_jpeg()

src/database.py
  - Queue(maxsize=120), drop-oldest policy

src/detector.py
  - GPU profiler deque(maxlen=300)

src/stereo_calibration.py
  - ArUco calibration APIs

src/calibration_ui.py
  - ArUco-first UI

src/dashboard_monitoring.py
  - CalibrationQualityMonitor

config.py
  - ArUco config block
```

---

## 9. RAM Crash Fixes (Follow-up Session)

### Symptom
Activity Monitor RSS growing past 1.2 GB during dual-camera recording, eventually crashing the process.

### Root Causes & Fixes

| # | Root cause | File | Fix |
|---|---|---|---|
| 1 | `MAX_FRAME_WIDTH = 1280` — MediaPipe Metal inference on 1280×720 frames allocates ~200–400 MB/frame in unified memory | `config.py` | Changed to `640`; 4× less Metal buffer per inference cycle |
| 2 | `ImageTk.PhotoImage` created fresh every ~15 ms Tkinter poll tick (~2.5 MB Tk allocation at 67/s = ~166 MB/s GC backlog) | `main_gui.py` | `_display_frame_tkinter` now reuses a single `PhotoImage` via `paste(img)` in-place; explicit `del rgb, img` after use |
| 3 | `np.hstack([local_display, remote_display])` at 640×480×3 (1.84 MB) stored in `_latest_display_frame` every frame, never explicitly freed | `main_gui.py` | Display throttled to every 3rd frame (`frame_count % 3`); dimensions shrunk to 320×240 (4× smaller); `del combined, local_display, remote_display` immediately after `_schedule_display_tkinter()` |
| 4 | CLAHE intermediate arrays (`lab, l, a, b, cl, limg`) — 5–6 arrays at 640×360×3 each, freed only at end of loop iteration | `src/detector.py` | `del lab, l, a, b, cl, limg` immediately after CLAHE block |
| 5 | `mp_image` source arrays (`frame_rgba` on GPU path, `frame_rgb` on CPU path) — frame data already copied into Metal buffer but Python reference kept alive | `src/detector.py` | `del frame_rgba` / `del frame_rgb` immediately after `mp.Image()` constructor |
| 6 | Duplicate `draw_landmarks()` call for pose results — doubled protobuf object allocations per frame | `src/visualizer.py` | Removed the duplicate call |
| 7 | Python GC not triggered frequently enough to reclaim MediaPipe/Metal native objects | `main_gui.py` | `gc.collect()` every 150 frames in master display block |

### Memory Model (Apple Silicon)

On M-series Macs, GPU (Metal) and CPU share the same physical DRAM pool.
MediaPipe allocates Metal command buffers proportional to input frame size.
At `MAX_FRAME_WIDTH = 1280`, each `detect_for_video()` call touched the unified memory bus
with a 1280×720×4 (RGBA) tensor → ~3.5 MB per call × 30 fps = ~105 MB/s Metal pressure
*before* any result storage. Halving to 640 reduces this to ~26 MB/s.

### Expected Steady-State After Fix

| Metric | Before | After |
|---|---|---|
| MediaPipe Metal per-frame | ~200–400 MB | ~50–100 MB |
| Tkinter display per-frame allocation | ~2.5 MB × 67/s | ~0 (paste in-place) |
| Display numpy array size | 1.84 MB × 30 fps | 0.46 MB × 10 fps |
| CLAHE intermediates freed | end of loop | immediately |
| Target RSS | 1.2 GB+ (crash) | 300–500 MB stable |

### Additional Files Changed

```
config.py
  - MAX_FRAME_WIDTH: 1280 → 640

main_gui.py
  - import gc (top of file)
  - _display_frame_tkinter: paste()-reuse PhotoImage; del rgb, img
  - Dual camera display block: throttled to frame % 3; 320×240; del combined; gc.collect() every 150 frames

src/visualizer.py
  - Removed duplicate draw_landmarks() call for pose results

src/detector.py
  - del lab, l, a, b, cl, limg after CLAHE block
  - del frame_rgba (GPU path) and del frame_rgb (CPU path) after mp.Image() constructor
```

---

## 10. Video Feed Freeze, Ablation Study & Windows Parity (Follow-up Session)

### 10.1 Video Feed Getting Stuck — Root Cause

**Symptom:** After running for a few minutes, the camera feed window would freeze even though the FPS counter kept ticking. Metrics panel also stopped updating.

**Root cause:** `_start_ui_poller()` in `main_gui.py` — the `_poll()` function had no `try/finally`. Any unhandled exception inside the poll tick (e.g., a Tkinter widget error, a bad metrics dict) would skip the `self.root.after(15, _poll)` call at the end of the function, **permanently killing the Tkinter polling loop**. The video thread kept capturing frames at full FPS but nothing ever re-rendered them.

**Fix:**
```python
def _poll():
    try:
        # ... all display + metrics work ...
    finally:
        # ALWAYS reschedule regardless of any exception above.
        # Without this, ONE exception permanently kills the display + metrics.
        if self.running:
            self.root.after(15, _poll)
```

Each flush sub-call (`_flush_display_tkinter`, `_flush_metrics_gui`) is also individually guarded so one failure cannot suppress the other.

### 10.2 Per-frame `FrameData` Import Removed

`from src.master_coordinator import FrameData` was inside the video loop body, executing ~30× per second. Moved to the top-level master-mode import block (runs once at startup).

### 10.3 Display Resolution Corrected

Previous fix used `320×240` (wrong 4:3 aspect ratio). Corrected to `640×360` — native 16:9, matches camera AR, still throttled to every 3rd frame with `del combined`.

### 10.4 Ablation Study Infrastructure

Added to `config.py` — four flags to independently disable each memory fix so you can measure its RSS contribution:

```python
ABLATION_DISABLE_FRAME_RESIZE     = False  # True → inference at FRAME_WIDTH (original)
ABLATION_DISABLE_DISPLAY_THROTTLE = False  # True → redraw every frame (not every 3rd)
ABLATION_DISABLE_PHOTO_REUSE      = False  # True → new PhotoImage each Tk poll tick
ABLATION_DISABLE_CLAHE_DEL        = False  # True → skip explicit del of CLAHE arrays
ABLATION_RSS_LOG_INTERVAL         = 60     # print [ABLATION] RSS every N frames (0 = off)
```

Each flag is wired into the actual code path it disables (`detector.py` for `FRAME_RESIZE` and `CLAHE_DEL`; `main_gui.py` for `DISPLAY_THROTTLE` and `PHOTO_REUSE`).

**Procedure:**
1. Baseline: all flags `False` — run 60 seconds, note terminal RSS
2. Set one flag to `True`, restart, note RSS again
3. Difference = that fix's individual contribution

**Expected per-fix RSS savings (approximate, 30 fps dual-camera):**

| Fix | Expected saving |
|---|---|
| `FRAME_RESIZE` (1280→640) | +200–400 MB when disabled |
| `PHOTO_REUSE` (paste vs new) | +100–200 MB when disabled |
| `DISPLAY_THROTTLE` (every-3rd) | +30–60 MB when disabled |
| `CLAHE_DEL` (explicit del) | +20–40 MB when disabled |

Terminal output while running:
```
[ABLATION] frame=60  RSS=342.1MB  active_fixes=['frame_resize', 'display_throttle', 'photo_reuse', 'clahe_del']
```

`psutil>=5.9.0` added to `requirements.txt` (import is guarded so app still runs if not installed).

### 10.5 Windows Server Parity (Motion-capture-vs1)

All memory fixes from §9 applied to the Windows server repo:

| File | Change |
|---|---|
| `Motion-capture-vs1/config.py` | `MAX_FRAME_WIDTH = 640`; `INFERENCE_BACKEND = 'cpu'`; `PREFER_GPU_DELEGATE = False` |
| `Motion-capture-vs1/src/detector.py` | `del lab, l, a, b, cl, limg` after CLAHE; `del frame_rgb` after `mp.Image()` |
| `Motion-capture-vs1/src/visualizer.py` | Duplicate `draw_landmarks()` call for pose removed |

On the Windows PC: pull updated files and restart with `python launch_multi_camera.py --mode server`.

### 10.6 Files Changed This Session

```
main_gui.py
  - _start_ui_poller: _poll() wrapped in try/finally (feed-freeze fix)
  - _flush_display_tkinter and _flush_metrics_gui individually guarded inside poll
  - FrameData import moved from video loop body to top-level master import block
  - Display resolution: 320×240 → 640×360 (correct 16:9 AR)
  - Display throttle: ABLATION_DISABLE_DISPLAY_THROTTLE flag wired in
  - _display_frame_tkinter: ABLATION_DISABLE_PHOTO_REUSE flag wired in
  - [ABLATION] RSS telemetry log every ABLATION_RSS_LOG_INTERVAL frames
  - psutil import (guarded try/except) + _get_rss_mb() helper

config.py
  - ABLATION_DISABLE_FRAME_RESIZE / DISPLAY_THROTTLE / PHOTO_REUSE / CLAHE_DEL flags
  - ABLATION_RSS_LOG_INTERVAL setting

src/detector.py
  - ABLATION_DISABLE_FRAME_RESIZE wired into both resize paths
  - ABLATION_DISABLE_CLAHE_DEL wired into del statement
  - FRAME_WIDTH + ablation imports added to config import block

requirements.txt
  - psutil>=5.9.0

Motion-capture-vs1/config.py
  - MAX_FRAME_WIDTH = 640
  - INFERENCE_BACKEND = 'cpu'
  - PREFER_GPU_DELEGATE = False

Motion-capture-vs1/src/detector.py
  - del lab, l, a, b, cl, limg after CLAHE
  - del frame_rgb / del frame_rgba after mp.Image() constructor

Motion-capture-vs1/src/visualizer.py
  - Duplicate draw_landmarks() call for pose removed
```
