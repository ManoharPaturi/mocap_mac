# Changes Reference: Mac (done) → Windows (to apply)

This file tracks every change applied to `Motion-capture/` (Mac) on the
`copilot/fix-live-video-buffering` branch and the **exact equivalent** that
must be manually applied to the Windows machine's `Motion-capture-vs1/` repo.

---

## 1. `config.py`

### Mac change (already applied)
```python
# Was:
NETWORK_JPEG_QUALITY = 30
SYNC_DYNAMIC_FACTOR = 0.5
# (no SYNC_THRESHOLD_MIN_MS / SYNC_THRESHOLD_MAX_MS)
SYNC_TIME_THRESHOLD_MS = 200.0
SYNC_DYNAMIC_THRESHOLD_ENABLED = False
FRAME_BUFFER_SIZE = 30

# Now:
NETWORK_JPEG_QUALITY = 45
SYNC_DYNAMIC_FACTOR = 0.6
SYNC_THRESHOLD_MIN_MS = 20
SYNC_THRESHOLD_MAX_MS = 30
SYNC_TIME_THRESHOLD_MS = 200.0          # unchanged
SYNC_DYNAMIC_THRESHOLD_ENABLED = False  # unchanged
FRAME_BUFFER_SIZE = 30                  # unchanged
```

### Windows — apply to `Motion-capture-vs1/config.py`
```python
# Current Windows values (need changing):
NETWORK_JPEG_QUALITY = 35     → change to 45
SYNC_TIME_THRESHOLD_MS = 50.0 → change to 200.0
FRAME_BUFFER_SIZE = 2         → change to 30

# Add these new lines (they don't exist yet):
SYNC_DYNAMIC_THRESHOLD_ENABLED = False
SYNC_DYNAMIC_FACTOR = 0.6
SYNC_THRESHOLD_MIN_MS = 20
SYNC_THRESHOLD_MAX_MS = 30
```

---

## 2. `src/camera.py`

### Mac change (already applied)
- Added `import sys`
- Added `CAP_DSHOW` on Windows (`sys.platform == 'win32'`) — no-op on Mac
- Added `self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)` right after `VideoCapture()`
- Added `self.frame_event = threading.Event()` to `__init__`
- `update()` thread calls `self.frame_event.set()` after every successful grab
- `read()` calls `self.frame_event.clear()` before returning frame
- Added `wait_for_frame(timeout=0.033)` method that wraps `frame_event.wait()`
- `release()` calls `self.frame_event.set()` to unblock any waiting thread

### Windows — apply to `Motion-capture-vs1/src/camera.py`
The Windows camera.py uses `CAP_DSHOW` already. Apply these diffs:

```python
# After: self.cap = cv2.VideoCapture(camera_id, cv2.CAP_DSHOW)
# Add immediately below:
self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

# In __init__, change:
self.grabbed = False
self.frame = None
self.running = False
# To:
self.grabbed = False
self.frame = None
self.frame_event = threading.Event()
self.running = False

# In update(), change:
                if grabbed:
                    self.grabbed = grabbed
                    self.frame = frame
                else:
                    self.running = False
            else:
                self.running = False
            time.sleep(0.005)   # ← DELETE this line
# To:
                if grabbed:
                    self.grabbed = grabbed
                    self.frame = frame
                    self.frame_event.set()   # ← ADD
                else:
                    self.running = False
            else:
                self.running = False
            # (no sleep — event-driven now)

# Change read() from:
def read(self):
    """Return the most recent frame."""
    return self.frame if self.grabbed else None
# To:
def read(self):
    """Return the most recent frame."""
    self.frame_event.clear()
    return self.frame if self.grabbed else None

# Add new method after read():
def wait_for_frame(self, timeout=0.033):
    """Block until a new frame is available or timeout elapses."""
    return self.frame_event.wait(timeout)

# Change release() from:
def release(self):
    """Release the camera resource."""
    self.running = False
    if self.thread:
        self.thread.join()
# To:
def release(self):
    """Release the camera resource."""
    self.running = False
    self.frame_event.set()   # unblock any waiting thread
    if self.thread:
        self.thread.join()
```

---

## 3. `src/streamer.py`

### Mac change (already applied)
Added `self.camera.wait_for_frame()` before `camera.read()` in `_process_loop`.

### Windows — apply to `Motion-capture-vs1/src/streamer.py`
```python
# Change _process_loop from:
    def _process_loop(self):
        while self.running and self.camera.is_opened():
            frame = self.camera.read()
# To:
    def _process_loop(self):
        while self.running and self.camera.is_opened():
            # Block until a fresh frame is captured; avoids re-processing stale frames
            self.camera.wait_for_frame()
            frame = self.camera.read()
```

---

## 4. `main_gui.py` — video_loop `camera.read()` call

### Mac change (already applied)
Mac `video_loop` already calls `self.camera.wait_for_frame(timeout=0.033)` before `camera.read()`.

### Windows — apply to `Motion-capture-vs1/main_gui.py`
```python
# Change video_loop from:
    def video_loop(self):
        while self.running:
            frame = self.camera.read()
# To:
    def video_loop(self):
        while self.running:
            # Block until a fresh frame is captured; avoids re-processing stale frames
            self.camera.wait_for_frame(timeout=0.033)
            frame = self.camera.read()
```
> ✅ Already applied in the local `Motion-capture-vs1/` copy — copy this to the Windows machine.

---

## 5. `main_gui.py` — FPS label thread safety

### Mac change (already applied)
```python
# Was (called directly from video_loop background thread — unsafe):
self.fps_label.config(text=f"FPS: {fps:04.1f}")

# Now (enqueued to run on Tk main thread):
self._enqueue_ui_task(self.fps_label.config, text=f"FPS: {fps:04.1f}")
```

### Windows — apply to `Motion-capture-vs1/main_gui.py`
Find this block in `video_loop`:
```python
            # Update FPS label via main-thread queue — never call .config() from video thread
            try:
                self._enqueue_ui_task(self.fps_label.config, text=f"FPS: {fps:04.1f}")
            except Exception:
                pass
```
> ✅ Already correct in the local `Motion-capture-vs1/` copy — copy to Windows machine.

---

## 6. `main_gui.py` — `_master_result_queue` maxsize

### Mac change (already applied)
```python
# Was:
self._master_result_queue = queue.Queue(maxsize=20)
# Now:
self._master_result_queue = queue.Queue(maxsize=2)
```
> Windows (`server` mode) does not have a `_master_result_queue` — skip this.

---

## 7. `main_gui.py` — Async DB save worker

### Mac change (already applied)
Added `_db_save_queue` + `_db_save_worker` thread in `__init__`. Moved all
`db.save_synced_frame()` and `coordinator.save_frame_to_database()` calls in
`_drain_master_results()` to enqueue onto `_db_save_queue` instead. Added
graceful shutdown in `cleanup()`.

### Windows
Windows runs in `server` mode and does not call `save_synced_frame` or
`_drain_master_results`. **No change needed** for this item.

---

## 8. `main_gui.py` — `received_at` fix for local_cam

### Mac change (already applied)
```python
# Was (timestamp in nanoseconds — wrong units for received_at):
received_at=timestamp
# Now (Mac wall-clock seconds — same units as remote camera's received_at):
received_at=time.time()
```

### Windows — apply to `Motion-capture-vs1/main_gui.py`
Find the `FrameData(...)` block in `video_loop` (master mode section):
```python
# Change:
                    received_at=timestamp  # Add received_at (same as timestamp for local)
# To:
                    received_at=time.time()  # wall-clock seconds — same units as remote cam received_at
```
> ✅ Already applied in the local copy — copy to Windows machine.

---

## 9. `main_gui.py` — Dual W:|M: dashboard display

### Mac (`Motion-capture/main_gui.py`) — already implemented
- `angle_labels` uses `val_width=22`, `val_font=("Courier", 9, "bold")` in master mode
- `update_table_safe(data)` accepts `{'local':..., 'remote':...}` dict, inserts MAC + WIN rows
- `update_metrics_gui(metrics)` shows `M:{local} | W:{remote}` for master mode
- `_drain_master_results()` passes `{'local': self.latest_local_metrics, 'remote': self.latest_remote_metrics}`

### Windows — apply to `Motion-capture-vs1/main_gui.py`
All 7 sub-changes below are **already applied in the local copy** — copy to Windows machine.

**9a. `__init__` — add state vars** (after `self.prev_time = None`):
```python
self.latest_local_metrics = {}
self.latest_remote_metrics = {}
```

**9b. `setup_gui` — widen angle_labels** (replace the `val_label = tk.Label(...)` block):
```python
val_font = ("Courier", 9, "bold") if MULTI_CAMERA_MODE == 'master' else ("Courier", 12, "bold")
val_width = 22 if MULTI_CAMERA_MODE == 'master' else 8
val_label = tk.Label(metrics_frame, text="--", bg='#0f0f1e', fg='#00ff88',
                     font=val_font, width=val_width, anchor='w')
val_label.grid(row=row, column=col*2+1, padx=10, pady=5, sticky="w")
self.angle_labels[key] = (val_label, unit)
```

**9c. `setup_gui` — rename treeview first column** (Source instead of Time, wider):
```python
self.tree.heading("Time", text="Source")
self.tree.column("Time", width=100, anchor=tk.CENTER)
```

**9d. `video_loop` — update `latest_local_metrics` and pass dict to `_schedule_metrics_gui`**:
```python
# After: self.prev_metrics = metrics  /  self.prev_time = now
# Add:
self.latest_local_metrics = metrics

if self.frame_count % 2 == 0:
    self._schedule_metrics_gui({
        'local': self.latest_local_metrics,
        'remote': self.latest_remote_metrics
    })
```

**9e. `video_loop` — pass dict to `update_table_safe`**:
```python
# Change:
self._enqueue_ui_task(self.update_table_safe, results)
# To:
self._enqueue_ui_task(self.update_table_safe, {
    'local': self.latest_local_metrics,
    'remote': self.latest_remote_metrics
})
```

**9f. `update_table_safe` — rewrite to handle dict**:
```python
def update_table_safe(self, data):
    local_metrics = {}
    remote_metrics = {}
    if isinstance(data, dict) and ('local' in data or 'remote' in data):
        local_metrics = data.get('local') or {}
        remote_metrics = data.get('remote') or {}
    else:
        local_metrics = self.prev_metrics  # fallback

    keys = ["Angle_Shoulder_L", "Angle_Shoulder_R",
            "Angle_Elbow_L", "Angle_Elbow_R",
            "Angle_Knee_L", "Angle_Knee_R",
            "Velocity_Angle_Elbow_R", "Velocity_Wrist_R"]

    def _insert_row(source_tag, metrics):
        row_values = ["-"] * 8
        for i, key in enumerate(keys):
            if key in metrics:
                row_values[i] = f"{metrics[key]:.1f}"
        ts = time.strftime("%H:%M:%S")
        try:
            self.tree.insert("", 0, values=(f"{source_tag} {ts}", *row_values))
        except Exception:
            pass

    if local_metrics:
        _insert_row("WIN", local_metrics)
    if remote_metrics:
        _insert_row("MAC", remote_metrics)

    children = self.tree.get_children()
    if len(children) > 10:
        self.tree.delete(children[-1])
```

**9g. `update_metrics_gui` — rewrite for W:|M: format**:
```python
def update_metrics_gui(self, metrics):
    """Update angle labels — shows W: | M: dual format in master mode."""
    local_metrics = metrics
    remote_metrics = {}
    if isinstance(metrics, dict) and ('local' in metrics or 'remote' in metrics):
        local_metrics = metrics.get('local') or {}
        remote_metrics = metrics.get('remote') or {}

    for key, (label_widget, unit) in self.angle_labels.items():
        local_val = local_metrics.get(key) if isinstance(local_metrics, dict) else None
        remote_val = remote_metrics.get(key) if isinstance(remote_metrics, dict) else None

        if MULTI_CAMERA_MODE == 'master':
            local_txt = f"{local_val:.2f}{unit}" if local_val is not None else "--"
            remote_txt = f"{remote_val:.2f}{unit}" if remote_val is not None else "--"
            label_widget.config(text=f"W:{local_txt} | M:{remote_txt}")
        elif local_val is not None:
            label_widget.config(text=f"{local_val:.2f}{unit}")
        else:
            label_widget.config(text="0.0")
```

---

## 10. `main_gui.py` — UI task queue + 15ms poller (thread safety overhaul)

### Mac — already implemented in full

### Windows — already applied in local copy
All of the following are **already in `Motion-capture-vs1/main_gui.py`** locally.
Just copy the file to the Windows machine:

- `self._ui_task_queue = queue.Queue(maxsize=256)` in `__init__`
- `_start_ui_poller()` called after `setup_gui()` in `__init__`
- `_enqueue_ui_task(func, *args, **kwargs)` method added
- `_start_ui_poller()` method: fixed 15ms `root.after` loop draining `_ui_task_queue`,
  flushing display + metrics, rescheduling via try/finally
- `_display_frame_tkinter()` reuses `self._cam_photo.paste(img)` instead of
  creating a new `ImageTk.PhotoImage` every frame
- `_schedule_display_tkinter()` just sets flags (no `root.after(0,...)` call)
- `_schedule_metrics_gui()` just sets flags (no `root.after(0,...)` call)
- `fps_label.config(...)` enqueued via `_enqueue_ui_task`
- `update_table_safe(...)` call enqueued via `_enqueue_ui_task`
- Display throttled to every 3rd frame (~10 fps) inside `video_loop`

---

## Summary: What is already in the local `Motion-capture-vs1/` copy

Everything in sections 4, 5, 8, 9, 10 is **already in the local Windows repo copy**
(`/Users/manoharpaturi/Mrudula/Motion-capture-vs1/`).

**To apply to the Windows machine**: copy these files from the local repo to Windows:
- `main_gui.py`
- `src/camera.py`
- `src/streamer.py`

**Also manually edit on Windows** (config changes not committed to vs1):
- `config.py`:
  - `NETWORK_JPEG_QUALITY = 35` → `45`
  - `SYNC_TIME_THRESHOLD_MS = 50.0` → `200.0`
  - `FRAME_BUFFER_SIZE = 2` → `30`
  - Add: `SYNC_DYNAMIC_THRESHOLD_ENABLED = False`
  - Add: `SYNC_DYNAMIC_FACTOR = 0.6`
  - Add: `SYNC_THRESHOLD_MIN_MS = 20`
  - Add: `SYNC_THRESHOLD_MAX_MS = 30`
