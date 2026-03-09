# Motion Capture — Changes Log

**Last Updated**: 9 March 2026  
**Repos**: `Motion-capture/` (Mac master) · `Motion-capture-vs1/` (Windows server)

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
