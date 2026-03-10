import cv2
import numpy as np
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import queue
import time
import platform
import os
import gc
try:
    import psutil as _psutil
    _PSUTIL_PID = os.getpid()
    def _get_rss_mb():
        return _psutil.Process(_PSUTIL_PID).memory_info().rss / 1_048_576
    def _get_peak_memory_mb():
        mi = _psutil.Process(_PSUTIL_PID).memory_info()
        # peak_wset on Windows, rss elsewhere (best available)
        return getattr(mi, 'peak_wset', mi.rss) / 1_048_576
    _RSS_IS_PEAK = False   # psutil gives current RSS
except ImportError:
    # psutil not installed — fall back to WinAPI (Windows) or resource (macOS/Linux)
    try:
        import ctypes as _ctypes, ctypes.wintypes as _wt
        class _PROCESS_MEMORY_COUNTERS(_ctypes.Structure):
            _fields_ = [
                ('cb', _wt.DWORD), ('PageFaultCount', _wt.DWORD),
                ('PeakWorkingSetSize', _ctypes.c_size_t),
                ('WorkingSetSize', _ctypes.c_size_t),
                ('QuotaPeakPagedPoolUsage', _ctypes.c_size_t),
                ('QuotaPagedPoolUsage', _ctypes.c_size_t),
                ('QuotaPeakNonPagedPoolUsage', _ctypes.c_size_t),
                ('QuotaNonPagedPoolUsage', _ctypes.c_size_t),
                ('PagefileUsage', _ctypes.c_size_t),
                ('PeakPagefileUsage', _ctypes.c_size_t),
            ]
        _psapi = _ctypes.windll.psapi
        _kernel32 = _ctypes.windll.kernel32
        _CURRENT_PROCESS = _kernel32.GetCurrentProcess()
        def _get_rss_mb():
            pmc = _PROCESS_MEMORY_COUNTERS(); pmc.cb = _ctypes.sizeof(pmc)
            _psapi.GetProcessMemoryInfo(_CURRENT_PROCESS, _ctypes.byref(pmc), pmc.cb)
            return pmc.WorkingSetSize / 1_048_576
        def _get_peak_memory_mb():
            pmc = _PROCESS_MEMORY_COUNTERS(); pmc.cb = _ctypes.sizeof(pmc)
            _psapi.GetProcessMemoryInfo(_CURRENT_PROCESS, _ctypes.byref(pmc), pmc.cb)
            return pmc.PeakWorkingSetSize / 1_048_576
        _RSS_IS_PEAK = False
    except Exception:
        # macOS / Linux stdlib fallback
        try:
            import resource as _resource
            _RSS_IS_PEAK = platform.system() == 'Darwin'  # peak on macOS, current on Linux
            def _get_rss_mb():
                ru = _resource.getrusage(_resource.RUSAGE_SELF)
                if platform.system() == 'Darwin':
                    return ru.ru_maxrss / 1_048_576   # bytes → MB on macOS
                return ru.ru_maxrss / 1024.0           # KB → MB on Linux
            def _get_peak_memory_mb():
                return _get_rss_mb()  # resource only has peak on macOS anyway
        except Exception:
            _RSS_IS_PEAK = False
            def _get_rss_mb():
                return 0.0
            def _get_peak_memory_mb():
                return 0.0
try:
    from PIL import Image as PILImage, ImageTk
    PIL_AVAILABLE = True
except ImportError:
    PIL_AVAILABLE = False
try:
    import torch
    import torchvision.transforms.functional as TF
    from torchvision.io import decode_jpeg
    TORCH_GUI_AVAILABLE = True
except Exception:
    torch = None
    TF = None
    decode_jpeg = None
    TORCH_GUI_AVAILABLE = False
from src.camera import Camera
from src.detector import MocapDetector
from src.visualizer import Visualizer
from src.database import MocapDB
from src.report_generator import ReportGenerator
from src.pose_corrector import PoseCorrector
from src.calculations import Calculations
import config
from config import (
    DRAW_LANDMARKS, MULTI_CAMERA_MODE, REMOTE_CAMERA_IP,
    CUDA_ENABLED, MPS_ENABLED, DEVICE, TORCH_AVAILABLE,
    ABLATION_DISABLE_DISPLAY_THROTTLE, ABLATION_DISABLE_PHOTO_REUSE,
    ABLATION_DISABLE_FRAME_RESIZE, ABLATION_DISABLE_CLAHE_DEL,
    ABLATION_RSS_LOG_INTERVAL
)
# Mac MPS + CUDA share the same tensor pipeline; only decode_jpeg differs
_GPU_AVAILABLE = (CUDA_ENABLED or MPS_ENABLED)

# Multi-camera imports (conditional)
if MULTI_CAMERA_MODE == 'server':
    from src.camera_server import CameraServer
elif MULTI_CAMERA_MODE == 'master':
    from src.master_coordinator import MasterCoordinator, FrameData
    from src.triangulation import Triangulator

class MocapGUI:
    def __init__(self):
        # Initialize components
        self.camera = Camera()
        self.detector = MocapDetector()
        self.visualizer = Visualizer()
        self.db = MocapDB() 
        self.reporter = ReportGenerator(self.db) 
        self.corrector = PoseCorrector() # Init Physics Engine
        
        # Multi-camera network components
        self.network_server = None
        self.coordinator = None
        self.network_server = None
        self.coordinator = None
        self.triangulator = None
        self.remote_frame = None  # Buffer for remote camera frame
        self._last_remote_frame_time = None  # Wall-clock time of most recent remote JPEG (for DISCONNECTED detection)
        
        # Initialize network based on mode
        if MULTI_CAMERA_MODE == 'server':
            from config import NETWORK_CAMERA_ID
            self.network_server = CameraServer(NETWORK_CAMERA_ID)
            self.network_server.start()
            print("[GUI] Camera Server started - broadcasting to network")
            
        elif MULTI_CAMERA_MODE == 'master':
            if not REMOTE_CAMERA_IP:
                print("[GUI] WARNING: REMOTE_CAMERA_IP not set in config!")
            else:
                self.coordinator = MasterCoordinator(num_cameras=2)
                if hasattr(self.coordinator, 'attach_database'):
                    self.coordinator.attach_database(self.db)
                self.coordinator.start()
                self.coordinator.discover_cameras_manual([REMOTE_CAMERA_IP])
                # Triangulator with no calibration for now (will load when available)
                self.triangulator = Triangulator(calibration=None)
                print(f"[GUI] Master Coordinator started - connecting to {REMOTE_CAMERA_IP}")
        
        # State
        self.running = True
        self.is_recording = False
        self.session_id = None
        self.frame_count = 0
        self._aruco_calib_ui = None  # active CalibrationUI instance (or None)

        # Tkinter camera display window (replaces cv2.imshow on macOS)
        self._cam_window = None
        self._cam_label = None
        self._cam_photo = None  # keep reference to avoid GC
        self._latest_display_frame = None
        self._latest_display_title = "Camera Feed"
        self._display_update_pending = False
        self._latest_metrics = None
        self._metrics_update_pending = False
        self._ui_task_queue = queue.Queue(maxsize=256)

        # Network send queue — latest-frame only to minimize streaming latency
        self._send_queue = queue.Queue(maxsize=1)
        self._send_worker_thread = threading.Thread(target=self._send_worker, daemon=True)
        self._send_worker_thread.start()

        # Remote JPEG decode queue — latest-only to keep display loop non-blocking
        self._remote_decode_queue = queue.Queue(maxsize=1)
        self._remote_decode_worker_thread = threading.Thread(target=self._remote_decode_worker, daemon=True)
        self._remote_decode_worker_thread.start()
        self._last_remote_gpu = None

        # Master pipeline queues: save/capture in video thread, compute/display in worker thread
        self._master_compute_queue = queue.Queue(maxsize=1)
        self._master_result_queue = queue.Queue(maxsize=2)
        self._master_compute_worker_thread = threading.Thread(target=self._master_compute_worker, daemon=True)
        self._master_compute_worker_thread.start()
        self._master_last_sync_batch = None

        # Async DB save queue — unbounded so no frame is ever dropped, drained by a
        # dedicated background thread so SQLite writes never stall the capture loop.
        self._db_save_queue = queue.Queue()
        self._db_save_worker_thread = threading.Thread(target=self._db_save_worker, daemon=True)
        self._db_save_worker_thread.start()

        # Thread-safe GUI State Caches (Initialize defaults)
        self.mirror_active = True
        self.markers_active = True
        self.roi_active = False
        self.face_det_active = True
        self.hand_det_active = True
        self.gamma_val = 1.0
        self.exposure_active = False
        self.model_type = "FULL"
        
        # Kinematics State
        self.prev_lm = []
        self.prev_metrics = {}
        self.prev_time = None
        self._metric_state = {
            'local_cam': {'prev_lm': [], 'prev_metrics': {}, 'prev_time': None}
        }
        self.latest_local_metrics = {}
        self.latest_remote_metrics = {}
        self.latest_quality = {}

        # Save-path tracing (debug): helps pinpoint where PC2/3D/KIN disappear.
        self._save_trace_enabled = True
        self._save_trace_every = 30
        self._save_trace = {
            'prequeue_synced': 0,
            'prequeue_mono': 0,
            'db_dequeue_synced': 0,
            'db_dequeue_mono': 0,
        }
        
        # Create tkinter window
        self.root = tk.Tk()
        
        # Set title based on mode
        mode_name = {
            'single': '',
            'server': ' - Camera Server (PC1)', 
            'master': ' - Master Coordinator (PC2)'
        }.get(MULTI_CAMERA_MODE, '')
        self.root.title(f"MoCap Live Dashboard{mode_name}")
        
        self.root.geometry("500x700")
        self.root.configure(bg='#0f0f1e')  # VS2 Dark Theme
        # Note: db and reporter already initialized above; do not re-init here (leaked connection)

        self.frame_count = 0 # Throttling counter
        
        
        # Setup GUI First (Important: Initialize vars before thread starts)
        self.setup_gui()
        self._start_ui_poller()
        
        # Mac OpenCV fix: Start window thread before video loop
        if platform.system() == 'Darwin':  # macOS
            try:
                # Some OpenCV builds on macOS can segfault in startWindowThread().
                # It is optional for this app because display is handled via Tkinter.
                if hasattr(cv2, 'startWindowThread'):
                    cv2.startWindowThread()
            except Exception as e:
                print(f"[GUI] Warning: cv2.startWindowThread skipped: {e}")
        
        # Start Video Thread
        self.running = True
        self.video_thread = threading.Thread(target=self.video_loop, daemon=True)
        self.video_thread.start()
        
    def _display_frame_tkinter(self, frame_bgr, title="Camera Feed"):
        """Display a BGR numpy frame in a tkinter Toplevel window (main-thread safe).
        Reuses a single persistent ImageTk.PhotoImage via paste() to avoid allocating
        a new Tk image object every frame (was the biggest source of uncollected memory).
        """
        if not PIL_AVAILABLE:
            return
        try:
            rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            img = PILImage.fromarray(rgb)
            del rgb  # free numpy array immediately; PIL wraps its buffer

            if self._cam_window is None or not self._cam_window.winfo_exists():
                # First-time window creation — allocate fresh PhotoImage
                self._cam_window = tk.Toplevel(self.root)
                self._cam_window.title(title)
                self._cam_window.configure(bg='black')
                self._cam_photo = ImageTk.PhotoImage(image=img)
                self._cam_label = tk.Label(self._cam_window, image=self._cam_photo, bg='black')
                self._cam_label.pack()
            else:
                # Update pixel data in-place — no new Python objects, no new Tk allocation.
                # paste() calls Tk's photo put command on the existing image handle.
                try:
                    if ABLATION_DISABLE_PHOTO_REUSE:
                        # Ablation: allocate fresh to measure paste() reuse savings
                        self._cam_photo = ImageTk.PhotoImage(image=img)
                        self._cam_label.configure(image=self._cam_photo)
                    else:
                        self._cam_photo.paste(img)
                except Exception:
                    # Size mismatch after a display-resolution change — recreate
                    self._cam_photo = ImageTk.PhotoImage(image=img)
                    self._cam_label.configure(image=self._cam_photo)

            self._cam_window.title(title)
            del img  # free PIL image immediately
        except Exception as e:
            if self.frame_count % 100 == 0:
                print(f"[Display] Frame render error: {e}")

    def _flush_display_tkinter(self):
        """Render only the latest queued frame on the Tk main thread."""
        self._display_update_pending = False
        frame = self._latest_display_frame
        title = self._latest_display_title
        if frame is not None:
            self._display_frame_tkinter(frame, title)

    def _schedule_display_tkinter(self, frame_bgr, title="Camera Feed"):
        """Queue newest frame for display; never let Tkinter callback backlog build."""
        self._latest_display_frame = frame_bgr
        self._latest_display_title = title
        self._display_update_pending = True

    def _flush_metrics_gui(self):
        """Apply latest metrics update on Tk main thread."""
        self._metrics_update_pending = False
        metrics = self._latest_metrics
        if metrics is not None:
            self.update_metrics_gui(metrics)

    def _schedule_metrics_gui(self, metrics):
        """Coalesce metrics updates so GUI never backlogs."""
        self._latest_metrics = metrics
        self._metrics_update_pending = True

    def _enqueue_ui_task(self, func, *args, **kwargs):
        """Thread-safe: enqueue a callable to run on Tk main thread."""
        try:
            self._ui_task_queue.put_nowait((func, args, kwargs))
        except queue.Full:
            # Drop oldest UI task to avoid unbounded queue growth.
            try:
                self._ui_task_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._ui_task_queue.put_nowait((func, args, kwargs))
            except Exception:
                pass
        except Exception:
            pass

    def _start_ui_poller(self):
        """Main-thread polling loop for queued UI tasks and coalesced updates.
        Uses try/finally to GUARANTEE rescheduling — any unhandled exception
        would otherwise permanently kill the poll loop and freeze the feed.
        """
        def _poll():
            try:
                # Execute queued tasks
                for _ in range(32):  # bound work per tick
                    try:
                        func, args, kwargs = self._ui_task_queue.get_nowait()
                    except queue.Empty:
                        break
                    try:
                        func(*args, **kwargs)
                    except Exception:
                        pass

                # Flush coalesced updates — each guarded so one failure can't kill the other
                if self._display_update_pending:
                    try:
                        self._flush_display_tkinter()
                    except Exception:
                        self._display_update_pending = False
                if self._metrics_update_pending:
                    try:
                        self._flush_metrics_gui()
                    except Exception:
                        self._metrics_update_pending = False
            finally:
                # ALWAYS reschedule regardless of any exception above.
                # Without this, ONE exception permanently kills the display + metrics.
                if self.running:
                    self.root.after(15, _poll)

        self.root.after(15, _poll)

    def _on_mousewheel(self, event):
        self.canvas.yview_scroll(int(-1*(event.delta/120)), "units")

    def setup_gui(self):
        # VS2 Style Styling
        style = ttk.Style()
        style.theme_use('clam')
        style.configure("Treeview", 
                        background="#1a1a2e",
                        foreground="#e0e0e0",
                        fieldbackground="#1a1a2e")
        style.configure("Treeview.Heading", 
                        background="#0f0f1e",
                        foreground="#00d4ff",
                        font=("Arial", 10, "bold"))
                        
        # --- SCROLLABLE CONTAINER ---
        # Create a container frame
        self.container = tk.Frame(self.root, bg='#0f0f1e')
        self.container.pack(fill="both", expand=True)

        # Create canvas
        self.canvas = tk.Canvas(self.container, bg='#0f0f1e', highlightthickness=0)
        
        # Create scrollbar
        self.scrollbar = ttk.Scrollbar(self.container, orient="vertical", command=self.canvas.yview)
        
        # Create scrollable frame INSIDE canvas
        self.scrollable_frame = tk.Frame(self.canvas, bg='#0f0f1e')

        # Configure scrollable frame
        self.scrollable_frame.bind(
            "<Configure>",
            lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all"))
        )

        # Draw frame on canvas
        self.canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw", width=480) # Width slightly less than window

        # Configure canvas
        self.canvas.configure(yscrollcommand=self.scrollbar.set)

        # Pack scrollbar and canvas
        self.scrollbar.pack(side="right", fill="y")
        self.canvas.pack(side="left", fill="both", expand=True)
        
        # Bind MouseWheel for scrolling
        self.root.bind_all("<MouseWheel>", self._on_mousewheel)
        
        # --- WIDGET PARENT CHANGED TO self.scrollable_frame ---
        parent = self.scrollable_frame
        
        # Header
        header_frame = tk.Frame(parent, bg='#0f0f1e')
        header_frame.pack(fill=tk.X, pady=20)
        
        tk.Label(header_frame, text="🎬 MoCap Live Dashboard", 
                 font=("Arial", 22, "bold"),
                 bg='#0f0f1e', fg='#00d4ff').pack(side=tk.LEFT, padx=20)
                 
        # Model Selector (Top Right)
        model_frame = tk.Frame(header_frame, bg='#0f0f1e')
        model_frame.pack(side=tk.RIGHT, padx=20)
        
        tk.Label(model_frame, text="Model:", bg='#0f0f1e', fg='#aaa', font=("Arial", 10)).pack(side=tk.LEFT, padx=5)
        self.model_var = tk.StringVar(value="FULL")
        self.model_combo = ttk.Combobox(model_frame, textvariable=self.model_var, 
                     values=["LITE", "FULL", "HEAVY"], 
                     state="readonly", width=8)
        self.model_combo.pack(side=tk.LEFT)
        self.model_combo.bind("<<ComboboxSelected>>", self.on_model_change)
        
        # Status Bar
        status_frame = tk.Frame(parent, bg='#1a1a2e', padx=10, pady=5)
        status_frame.pack(fill=tk.X, padx=20, pady=5)
        
        self.status_label = tk.Label(status_frame, text="● READY", 
                                     font=("Arial", 12, "bold"),
                                     bg='#1a1a2e', fg='#00ff88')
        self.status_label.pack(side=tk.LEFT)
        
        self.fps_label = tk.Label(status_frame, text="FPS: 00.0", 
                                 font=("Courier", 12),
                                 bg='#1a1a2e', fg='#e0e0e0')
        self.fps_label.pack(side=tk.RIGHT)
        
        # Live Data Table (Matches VS2 Data Preview)
        tk.Label(parent, text="📊 Live Data Feed ( Angles )", 
                 font=("Arial", 12), bg='#0f0f1e', fg='#F5F5DC').pack(anchor=tk.W, padx=20, pady=(20,5))
        
        # Columns: Time + Key Angles + Velocities
        cols = ("Time", "Angle_L_Shoulder", "Angle_R_Shoulder", "Angle_L_Elbow", "Angle_R_Elbow", "Angle_L_Knee", "Angle_R_Knee", "Vel_Elbow", "Vel_Wrist")
        self.tree = ttk.Treeview(parent, columns=cols, show='headings', height=8)
        
        self.tree.heading("Time", text="Time")
        self.tree.column("Time", width=60, anchor=tk.CENTER)
        
        headers = ["L Shldr", "R Shldr", "L Elbow", "R Elbow", "L Knee", "R Knee", "V Elb", "V Wri"]
        for col, title in zip(cols[1:], headers):
            self.tree.heading(col, text=title)
            self.tree.column(col, width=50, anchor=tk.CENTER)
            
        self.tree.pack(padx=20, fill=tk.X)
        

        
        # Live Biometrics Panel
        metrics_frame = tk.LabelFrame(parent, text="Live Biometrics (Degrees)", 
                                    bg='#0f0f1e', fg='#00d4ff', font=("Arial", 12, "bold"))
        metrics_frame.pack(fill=tk.X, padx=20, pady=10)
        
        self.angle_labels = {}
        # Format: (Label Text, Metric Key, Unit)
        metric_keys = [
            # Angles
            ("L Elbow", "Angle_Elbow_L", "°"), ("R Elbow", "Angle_Elbow_R", "°"),
            ("L Knee", "Angle_Knee_L", "°"),   ("R Knee", "Angle_Knee_R", "°"),
            ("L Shoulder", "Angle_Shoulder_L", "°"), ("R Shoulder", "Angle_Shoulder_R", "°"),
            # Lengths
            ("L Arm", "Length_UpperArm_L", ""), ("R Arm", "Length_UpperArm_R", ""),
            ("L Leg", "Length_UpperLeg_L", ""), ("R Leg", "Length_UpperLeg_R", "")
        ]
        
        for i, (label_text, key, unit) in enumerate(metric_keys):
            row = i // 2
            col = i % 2
            
            # Label
            tk.Label(metrics_frame, text=label_text, bg='#0f0f1e', fg='#e0e0e0', font=("Arial", 10)).grid(row=row, column=col*2, padx=10, pady=5, sticky="e")
            
            # Value — wider in master mode to fit "M:XXX.X° | W:XXX.X°" format
            val_font = ("Courier", 9, "bold") if MULTI_CAMERA_MODE == 'master' else ("Courier", 12, "bold")
            val_width = 22 if MULTI_CAMERA_MODE == 'master' else 8
            val_label = tk.Label(metrics_frame, text="--", bg='#0f0f1e', fg='#00ff88',
                                 font=val_font, width=val_width, anchor='w')
            val_label.grid(row=row, column=col*2+1, padx=4, pady=5, sticky="w")
            
            # Save widget and unit for updating
            self.angle_labels[key] = (val_label, unit)
            
        metrics_frame.columnconfigure(0, weight=1)
        metrics_frame.columnconfigure(1, weight=1)
        metrics_frame.columnconfigure(2, weight=1)
        metrics_frame.columnconfigure(3, weight=1)
        
        # Controls
        control_frame = tk.Frame(parent, bg='#0f0f1e')
        control_frame.pack(pady=10)
        
        # Toggles
        toggle_frame = tk.Frame(parent, bg='#0f0f1e')
        toggle_frame.pack(pady=5)
        
        self.mirror_var = tk.BooleanVar(value=True) # Default Mirror: ON
        tk.Checkbutton(toggle_frame, text="Mirror Camera", variable=self.mirror_var,
                       bg='#0f0f1e', fg='#e0e0e0', selectcolor='#0f0f1e',
                       activebackground='#0f0f1e', activeforeground='#e0e0e0',
                       command=self.update_toggles).pack(side=tk.LEFT, padx=10)
                       
        self.markers_var = tk.BooleanVar(value=True) # Default Markers: ON
        tk.Checkbutton(toggle_frame, text="Show Markers", variable=self.markers_var,
                       bg='#0f0f1e', fg='#e0e0e0', selectcolor='#0f0f1e',
                       activebackground='#0f0f1e', activeforeground='#e0e0e0',
                       command=self.update_toggles).pack(side=tk.LEFT, padx=10)
        
        self.record_btn = tk.Button(control_frame, text="▶ Start Capture",
                                    command=self.toggle_recording,
                                    font=("Arial", 14, "bold"),
                                    bg='#1a1a2e', fg='#00ff88',
                                    activebackground='#00d4ff',
                                    width=18, bd=0, relief=tk.FLAT)
        self.record_btn.pack(pady=10)

        # Imaging Controls (VS5)
        # Gamma, Exposure, Toggles
        image_frame = tk.LabelFrame(parent, text="Imaging & AI", 
                                    bg='#0f0f1e', fg='#00d4ff', font=("Arial", 10, "bold"))
        image_frame.pack(fill=tk.X, padx=20, pady=5)
        
        # Gamma
        tk.Label(image_frame, text="Gamma:", bg='#0f0f1e', fg='#aaa').grid(row=0, column=0, padx=5, sticky='e')
        self.gamma_var = tk.DoubleVar(value=1.0)
        gamma_scale = tk.Scale(image_frame, from_=1.0, to=1.4, resolution=0.1, 
                               variable=self.gamma_var, orient=tk.HORIZONTAL, 
                               bg='#0f0f1e', fg='#e0e0e0', highlightthickness=0,
                               command=self.update_imaging)
        gamma_scale.grid(row=0, column=1, sticky='w')
        
        # Features
        self.exposure_var = tk.BooleanVar(value=False)
        tk.Checkbutton(image_frame, text="Face Exposure", variable=self.exposure_var,
                       bg='#0f0f1e', fg='#e0e0e0', selectcolor='#0f0f1e',
                       activebackground='#0f0f1e', activeforeground='#e0e0e0',
                       command=self.update_imaging).grid(row=1, column=0, columnspan=2, sticky='w', padx=5)
        
        self.roi_var = tk.BooleanVar(value=False)
        tk.Checkbutton(image_frame, text="ROI Cropping", variable=self.roi_var,
                       bg='#0f0f1e', fg='#e0e0e0', selectcolor='#0f0f1e',
                       activebackground='#0f0f1e', activeforeground='#e0e0e0',
                       command=self.update_imaging).grid(row=2, column=0, columnspan=2, sticky='w', padx=5)
                       
        self.face_det_var = tk.BooleanVar(value=True)
        tk.Checkbutton(image_frame, text="Face Detect", variable=self.face_det_var,
                       bg='#0f0f1e', fg='#e0e0e0', selectcolor='#0f0f1e',
                       activebackground='#0f0f1e', activeforeground='#e0e0e0',
                       command=self.update_imaging).grid(row=3, column=0, sticky='w', padx=5)
                       
        self.hand_det_var = tk.BooleanVar(value=True)
        tk.Checkbutton(image_frame, text="Hand Detect", variable=self.hand_det_var,
                       bg='#0f0f1e', fg='#e0e0e0', selectcolor='#0f0f1e',
                       activebackground='#0f0f1e', activeforeground='#e0e0e0',
                       command=self.update_imaging).grid(row=3, column=1, sticky='w', padx=5)

        self.download_btn = tk.Button(control_frame, text="📥 Download Dataset",
                                      command=self.download_data,
                                      font=("Arial", 12),
                                      bg='#1a1a2e', fg='#e0e0e0',
                                      width=18, bd=0, relief=tk.FLAT)
        self.download_btn.pack(pady=5)
        
        self.viz_btn = tk.Button(control_frame, text="📊 Visualize Session",
                                      command=self.show_visualization,
                                      font=("Arial", 12),
                                      bg='#1a1a2e', fg='#00d4ff', # Cyan for Viz
                                      width=18, bd=0, relief=tk.FLAT)
        self.viz_btn.pack(pady=5)
        
        # ── Latency Panel (Master Mode) ──
        if MULTI_CAMERA_MODE == 'master':
            latency_frame = tk.LabelFrame(parent, text="⏱ Pipeline Latency",
                                          bg='#0f0f1e', fg='#ffa500',
                                          font=("Arial", 10, "bold"))
            latency_frame.pack(fill=tk.X, padx=20, pady=5)
            
            self.latency_labels = {}
            latency_stages = [
                ("Capture", "capture"), ("Detection", "detection"),
                ("Network", "network"), ("Sync", "sync"),
                ("Triangulation", "triangulation"), ("Total", "total")
            ]
            for i, (display_name, key) in enumerate(latency_stages):
                row = i // 3
                col = i % 3
                tk.Label(latency_frame, text=display_name, bg='#0f0f1e',
                         fg='#aaa', font=("Arial", 9)).grid(
                    row=row * 2, column=col, padx=8, pady=(3, 0), sticky='s')
                val_label = tk.Label(latency_frame, text="--", bg='#0f0f1e',
                                     fg='#ffa500', font=("Courier", 10, "bold"))
                val_label.grid(row=row * 2 + 1, column=col, padx=8, pady=(0, 3), sticky='n')
                self.latency_labels[key] = val_label
            latency_frame.columnconfigure(0, weight=1)
            latency_frame.columnconfigure(1, weight=1)
            latency_frame.columnconfigure(2, weight=1)
        else:
            self.latency_labels = {}

        # ── Reconstruction Quality Panel (Master Mode) ──
        if MULTI_CAMERA_MODE == 'master':
            quality_frame = tk.LabelFrame(parent, text="🎯 Reconstruction Quality",
                                          bg='#0f0f1e', fg='#00d4ff',
                                          font=("Arial", 10, "bold"))
            quality_frame.pack(fill=tk.X, padx=20, pady=5)

            self.quality_labels = {}
            quality_items = [
                ("Reproj", "reproj_error", "px"),
                ("Confidence", "confidence", ""),
                ("Residual", "residual_error", "px"),
                ("Uncertainty", "uncertainty", "m"),
                ("Calib RMS", "calibration_rms", "px"),
            ]
            for i, (display_name, key, unit) in enumerate(quality_items):
                row = i // 2
                col = i % 2
                tk.Label(quality_frame, text=display_name, bg='#0f0f1e',
                         fg='#aaa', font=("Arial", 9)).grid(
                    row=row * 2, column=col, padx=8, pady=(3, 0), sticky='s')
                val_label = tk.Label(quality_frame, text=f"--{unit}", bg='#0f0f1e',
                                     fg='#00d4ff', font=("Courier", 10, "bold"))
                val_label.grid(row=row * 2 + 1, column=col, padx=8, pady=(0, 3), sticky='n')
                self.quality_labels[key] = (val_label, unit)

            quality_frame.columnconfigure(0, weight=1)
            quality_frame.columnconfigure(1, weight=1)
        else:
            self.quality_labels = {}

        # ── Clock Sync Panel (Master Mode) ──
        if MULTI_CAMERA_MODE == 'master':
            clock_frame = tk.LabelFrame(parent, text="🕒 Clock Sync",
                                        bg='#0f0f1e', fg='#8bd3ff',
                                        font=("Arial", 10, "bold"))
            clock_frame.pack(fill=tk.X, padx=20, pady=5)

            self.clock_sync_status_label = tk.Label(
                clock_frame,
                text="Waiting for sync...",
                bg='#0f0f1e', fg='#8bd3ff',
                justify=tk.LEFT,
                anchor='w',
                font=("Courier", 9, "bold")
            )
            self.clock_sync_status_label.pack(fill=tk.X, padx=8, pady=6)
        else:
            self.clock_sync_status_label = None

        # ── Help Button ──
        help_frame = tk.Frame(parent, bg='#0f0f1e')
        help_frame.pack(fill=tk.X, padx=20, pady=5)
        
        help_btn = tk.Button(help_frame, text="❓ Metric Help",
                             command=self._show_help_dialog,
                             font=("Arial", 11),
                             bg='#1a1a2e', fg='#00d4ff',
                             width=18, bd=0, relief=tk.FLAT)
        help_btn.pack(side=tk.LEFT, padx=5)

        # Camera placement viewer (master mode)
        if MULTI_CAMERA_MODE == 'master':
            cam_viz_frame = tk.Frame(parent, bg='#0f0f1e')
            cam_viz_frame.pack(fill=tk.X, padx=20, pady=3)
            cam_viz_btn = tk.Button(cam_viz_frame, text="📷 Camera Setup Viewer",
                                    command=self._show_camera_setup,
                                    font=("Arial", 11),
                                    bg='#1a1a2e', fg='#00d4ff',
                                    width=22, bd=0, relief=tk.FLAT)
            cam_viz_btn.pack()

        # Initial attribute sync
        self.update_toggles()
        self.update_imaging()
        


    def update_toggles(self):
        """Cache simple toggles for thread safety."""
        try:
            self.mirror_active = self.mirror_var.get()
            self.markers_active = self.markers_var.get()
        except: pass

    def update_imaging(self, _=None):
        """Update detector imaging params from GUI."""
        try:
            # Cache values for thread safety check (optional, but good practice)
            self.gamma_val = self.gamma_var.get()
            self.exposure_active = self.exposure_var.get()
            self.face_det_active = self.face_det_var.get()
            self.hand_det_active = self.hand_det_var.get()
            self.roi_active = self.roi_var.get()
            
            self.detector.set_imaging_params(
                gamma=self.gamma_val,
                face_exposure=self.exposure_active,
                enable_face=self.face_det_active,
                enable_hand=self.hand_det_active,
                enable_roi=self.roi_active
            )
        except Exception as e:
            print(f"Error updating imaging: {e}")

    def on_model_change(self, event):
        """Handle model complexity change."""
        try:
            model_type = self.model_var.get()
            self.detector.reload(model_type)
        except Exception as e:
            print(f"Error reloading model: {e}")

    def _show_help_dialog(self):
        """Show a help dialog with metric definitions and equations."""
        help_win = tk.Toplevel(self.root)
        help_win.title("Metric Help & Glossary")
        help_win.geometry("520x600")
        help_win.configure(bg='#0f0f1e')
        
        # Scrollable text
        text_frame = tk.Frame(help_win, bg='#0f0f1e')
        text_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        scrollbar = tk.Scrollbar(text_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        text_widget = tk.Text(text_frame, wrap=tk.WORD, bg='#1a1a2e', fg='#e0e0e0',
                              font=("Courier", 11), yscrollcommand=scrollbar.set,
                              padx=10, pady=10)
        text_widget.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.config(command=text_widget.yview)
        
        # Tag for headers
        text_widget.tag_configure('header', foreground='#00d4ff', font=("Arial", 13, "bold"))
        text_widget.tag_configure('subheader', foreground='#ffa500', font=("Arial", 11, "bold"))
        text_widget.tag_configure('formula', foreground='#00ff88', font=("Courier", 11))
        
        help_content = [
            ("header", "MoCap Metric Reference\n\n"),
            ("subheader", "Joint Angles\n"),
            ("", "Computed via 3D arc-cosine of bone vectors.\n"),
            ("formula", "  θ = arccos( (A·B) / (|A|·|B|) )\n\n"),
            ("subheader", "Elbow Angle (L/R)\n"),
            ("", "Angle between upper arm and forearm.\n"
                 "Landmarks: Shoulder → Elbow → Wrist\n"
                 "Range: 0° (fully flexed) to 180° (fully extended)\n\n"),
            ("subheader", "Knee Angle (L/R)\n"),
            ("", "Angle between thigh and shin.\n"
                 "Landmarks: Hip → Knee → Ankle\n"
                 "Range: 0° (fully bent) to 180° (standing straight)\n\n"),
            ("subheader", "Shoulder Angle (L/R)\n"),
            ("", "Angle of upper arm relative to torso.\n"
                 "Landmarks: Hip → Shoulder → Elbow\n\n"),
            ("subheader", "Velocities\n"),
            ("", "Angular velocity = Δθ / Δt  (degrees/second)\n"
                 "Linear velocity  = Δp / Δt  (meters/second)\n\n"),
            ("formula", "  v = |p(t) - p(t-1)| / dt\n\n"),
            ("subheader", "Bone Lengths\n"),
            ("", "Euclidean distance between joint endpoints.\n"
                 "Normalized by body height estimate.\n\n"),
            ("subheader", "Confidence / Visibility\n"),
            ("", "Per-landmark confidence from MediaPipe (0-1).\n"
                 "Values below 0.5 are treated as unreliable.\n\n"),
            ("formula", "  conf = w_vis × avg_visibility + w_reproj × (1 - err/threshold)\n\n"),
            ("subheader", "3D Reconstruction Methods\n"),
            ("", "• triangulated: DLT from ≥2 camera views (best)\n"
                 "• monocular_X: Back-projected from single camera (fallback)\n"
                 "• world_landmarks: MediaPipe hip-relative coords (Tier 3)\n"
                 "• predicted: Held from last visible position (occluded)\n\n"),
            ("subheader", "Uncertainty (Disagreement)\n"),
            ("", "Max pairwise distance between per-camera monocular\n"
                 "estimates for the same landmark. Lower = more confident.\n\n"),
            ("formula", "  uncertainty = max‖p_A - p_B‖ for all camera pairs\n\n"),
            ("subheader", "Occlusion States\n"),
            ("", "• VISIBLE: Landmark seen by ≥1 camera\n"
                 "• PREDICTED: Using last known position (≤500ms)\n"
                 "• OCCLUDED: Lost for too long, dropped\n\n"),
            ("subheader", "1-Euro Filter\n"),
            ("", "Adaptive low-pass filter for jitter reduction.\n"
                 "min_cutoff: smoothness at rest (Hz)\n"
                 "beta: responsiveness during fast motion\n\n"),
            ("formula", "  cutoff = min_cutoff + beta × |dx/dt|\n"),
        ]
        
        for tag, content in help_content:
            if tag:
                text_widget.insert(tk.END, content, tag)
            else:
                text_widget.insert(tk.END, content)
        
        text_widget.config(state=tk.DISABLED)

    def _start_recalibration(self):
        """Launch the ArUco stereo calibration wizard."""
        self._launch_calibration_wizard()

    def _run_recalibration(self):
        """Legacy chessboard recalibration handler removed."""
        return

    def _on_aruco_calibration_saved(self, filepath: str):
        """Called after the ArUco wizard saves calibration.json.  Hot-reloads coordinator."""
        if self.coordinator and hasattr(self.coordinator, 'reload_calibration'):
            ok = self.coordinator.reload_calibration(filepath)
            if ok:
                messagebox.showinfo(
                    "Calibration Loaded",
                    f"Stereo calibration loaded into live session.\n"
                    f"Path: {filepath}\n"
                    f"RMS: {getattr(self.coordinator, 'calibration_rms_px', None)}"
                )
            else:
                messagebox.showwarning("Calibration Reload Failed",
                                       f"Calibration saved to {filepath} but failed to reload.\n"
                                       "Restart the app to pick it up.")
        self._aruco_calib_ui = None

    def _launch_calibration_wizard(self):
        """Launch the live ArUco stereo calibration wizard."""
        if not (MULTI_CAMERA_MODE == 'master'):
            messagebox.showinfo(
                "Master Mode Required",
                "ArUco stereo calibration requires Master mode (two cameras connected).\n"
                "Set MULTI_CAMERA_MODE = 'master' in config.py"
            )
            return

        if getattr(self, '_aruco_calib_ui', None) is not None:
            try:
                self._aruco_calib_ui.root.lift()
            except Exception:
                pass
            return

        try:
            from src.calibration_ui import CalibrationUI
            from config import CALIBRATION_FILE
        except ImportError as e:
            messagebox.showerror("Import Error", f"Cannot load calibration module:\n{e}")
            return

        calib_ui = CalibrationUI(
            parent=self.root,
            calibration_mode='aruco',
            output_path=CALIBRATION_FILE,
        )
        self._aruco_calib_ui = calib_ui

        # Patch save to trigger hot-reload after file is written
        original_save = calib_ui._save
        def _patched_save():
            # Grab path before calling original (which opens a dialog)
            import tkinter.filedialog as _fd
            original_save()
            # Reload from the configured output path (user may have changed it in dialog)
            saved_path = calib_ui.calibrator.result is not None and CALIBRATION_FILE
            if saved_path and os.path.exists(saved_path):
                self.root.after(200, lambda: self._on_aruco_calibration_saved(saved_path))
        calib_ui._save = _patched_save

        # Feed live frames into the wizard at ~10 fps via a recurring Tk callback
        def _tick_calibration():
            ui = getattr(self, '_aruco_calib_ui', None)
            if ui is None:
                return
            try:
                ui.root.winfo_exists()  # raises if destroyed
            except Exception:
                self._aruco_calib_ui = None
                return

            if ui._running and ui._capture_requested:
                # Grab local frame
                local_frame = getattr(self, '_latest_display_frame', None)
                # Grab latest remote JPEG decoded to numpy
                remote_frame = None
                if self.coordinator:
                    remote_ids = [
                        cid for cid in self.coordinator.frame_buffers
                        if cid != 'local_cam'
                    ]
                    if remote_ids:
                        jpeg = self.coordinator.get_latest_camera_jpeg(remote_ids[0])
                        if jpeg is not None:
                            try:
                                buf = np.frombuffer(jpeg, dtype=np.uint8)
                                remote_frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
                            except Exception:
                                remote_frame = None
                if local_frame is not None:
                    ui.process_frames(local_frame, remote_frame)

            # Reschedule
            if getattr(self, '_aruco_calib_ui', None) is not None:
                self.root.after(100, _tick_calibration)

        # Start preview automatically and begin tick
        calib_ui._toggle_preview()
        self.root.after(100, _tick_calibration)
        calib_ui.root.protocol("WM_DELETE_WINDOW", lambda: (
            calib_ui._close(),
            setattr(self, '_aruco_calib_ui', None)
        ))
        # Bring wizard to front
        calib_ui.root.lift()

    def _update_latency_panel(self):
        """Update latency labels from coordinator stats (called from main thread)."""
        if not self.latency_labels or not self.coordinator:
            return
        try:
            stats = self.coordinator.get_latency_stats()
            for stage, label in self.latency_labels.items():
                if stage in stats:
                    ms = stats[stage]['mean_ms']
                    label.config(text=f"{ms:.1f}ms")
                else:
                    label.config(text="--")
        except Exception:
            pass

    def _extract_quality_metrics(self, pose_3d: dict) -> dict:
        """Extract aggregate quality metrics from fused pose output."""
        if not pose_3d:
            return {}

        pose_points = pose_3d.get('pose_3d', {}) or {}
        uncertainty_map = pose_3d.get('uncertainty', {}) or {}

        reproj_vals = []
        confidence_vals = []
        for point in pose_points.values():
            if not isinstance(point, dict):
                continue
            method = point.get('method', '')
            if method == 'triangulated':
                if point.get('reproj_error') is not None:
                    reproj_vals.append(float(point.get('reproj_error', 0.0)))
                if point.get('visibility') is not None:
                    confidence_vals.append(float(point.get('visibility', 0.0)))

        uncertainty_vals = [float(v) for v in uncertainty_map.values() if v is not None]

        mean_reproj = (sum(reproj_vals) / len(reproj_vals)) if reproj_vals else None
        mean_conf = (sum(confidence_vals) / len(confidence_vals)) if confidence_vals else None
        mean_uncertainty = (sum(uncertainty_vals) / len(uncertainty_vals)) if uncertainty_vals else None

        summary = pose_3d.get('uncertainty_summary', {}) or {}
        summary_grade = summary.get('quality_grade')
        summary_mean_unc = summary.get('mean_uncertainty_m')

        calibration_rms = None
        if self.coordinator and hasattr(self.coordinator, 'get_calibration_quality_metrics'):
            try:
                cal_metrics = self.coordinator.get_calibration_quality_metrics()
                calibration_rms = cal_metrics.get('calibration_effective_error_px')
            except Exception:
                calibration_rms = None

        # Residual error is represented by reprojection residual in this pipeline.
        return {
            'reproj_error': mean_reproj,
            'confidence': mean_conf,
            'residual_error': mean_reproj,
            'uncertainty': mean_uncertainty,
            'calibration_rms': calibration_rms,
            'uncertainty_summary_m': summary_mean_unc,
            'quality_grade': summary_grade,
        }

    def _update_quality_panel(self):
        """Update reconstruction quality labels from latest fused pose metrics."""
        if not self.quality_labels:
            return

        metrics = self.latest_quality or {}
        for key, (label, unit) in self.quality_labels.items():
            value = metrics.get(key)
            if value is None:
                label.config(text=f"--{unit}")
                continue

            if key == 'confidence':
                label.config(text=f"{value:.3f}{unit}")
            elif key == 'uncertainty':
                label.config(text=f"{value:.4f}{unit}")
            else:
                label.config(text=f"{value:.2f}{unit}")

    def _update_clock_sync_panel(self):
        """Update clock synchronization diagnostics in master dashboard."""
        if not self.clock_sync_status_label or not self.coordinator:
            return
        try:
            if not getattr(config, 'ENABLE_CLOCK_SYNC', False):
                self.clock_sync_status_label.config(text="Clock sync: disabled")
                return

            status = self.coordinator.get_clock_sync_status() if hasattr(self.coordinator, 'get_clock_sync_status') else {}
            if not status:
                self.clock_sync_status_label.config(text="Clock sync: no camera offsets yet")
                return

            lines = []
            for cam_id in sorted(status.keys()):
                item = status.get(cam_id, {})
                lines.append(
                    f"{cam_id:10s} offset={item.get('offset_ms', 0.0):+7.2f}ms  "
                    f"drift={item.get('drift_ms_per_min', 0.0):+6.3f}ms/min  "
                    f"rtt={item.get('rtt_min_ms', 0.0):5.2f}ms  "
                    f"age={item.get('last_sync_age_s', -1.0):5.1f}s"
                )

            self.clock_sync_status_label.config(text="\n".join(lines))
        except Exception:
            pass

    def _launch_calibration_wizard(self):
        """ArUco stereo calibration wizard — delegate to _start_recalibration."""
        self._start_recalibration()

    def _show_camera_setup(self):
        """Show 3D camera placement viewer."""
        try:
            from src.camera_setup_viewer import CameraSetupViewer
            cal = None
            if self.coordinator and self.coordinator.triangulator:
                cal = self.coordinator.triangulator.calibration
            if not cal:
                messagebox.showinfo("Camera Setup",
                                    "No calibration loaded.\nLoad or run calibration first.")
                return
            viewer = CameraSetupViewer(cal)
            viewer.show()
            
            # Also print camera info
            info = viewer.get_camera_info()
            for cam_id, ci in info.items():
                pos = ci['position']
                euler = ci['euler_deg']
                print(f"[CameraSetup] {cam_id}: pos=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})  "
                      f"FOV={ci['fov_h_deg']:.0f}°×{ci['fov_v_deg']:.0f}°  "
                      f"yaw={euler['yaw']:.1f}° pitch={euler['pitch']:.1f}°")
                if 'baseline_to' in ci:
                    for other, bl in ci['baseline_to'].items():
                        print(f"  baseline to {other}: {bl:.3f}m")
        except Exception as e:
            messagebox.showerror("Camera Setup", f"Error: {e}")

    def show_visualization(self):
        """Generate static analysis report only (dashboard visualization removed)."""
        try:
            path = self.reporter.generate_report()
            if path:
                messagebox.showinfo("Report Ready", f"Full analysis saved to:\n{path}")
                
        except Exception as e:
            messagebox.showerror("Error", f"Viz failed: {e}")

    def toggle_recording(self):
        if not self.is_recording:
            try:
                recording_mode = 'stereo' if MULTI_CAMERA_MODE == 'master' else 'single'
                self.session_id = self.db.start_recording(recording_mode=recording_mode)
                if self.coordinator and hasattr(self.coordinator, 'reset_layer_frame_index'):
                    self.coordinator.reset_layer_frame_index()
                if hasattr(self.db, 'save_processing_metadata'):
                    sync_threshold_ms = float(config.SYNC_TIME_THRESHOLD_MS)
                    if self.coordinator and hasattr(self.coordinator, 'get_current_sync_threshold_ms'):
                        sync_threshold_ms = float(self.coordinator.get_current_sync_threshold_ms())
                    self.db.save_processing_metadata(
                        sync_threshold_ms=sync_threshold_ms,
                        sync_strategy='timestamp_match',
                        clock_sync_enabled=bool(getattr(config, 'ENABLE_CLOCK_SYNC', False)),
                        clock_sync_interval_sec=float(getattr(config, 'CLOCK_SYNC_INTERVAL_SEC', 0.0)),
                        triangulation_method='opencv_triangulate',
                        triangulation_version='1.0',
                        smoothing_version='1.0',
                        calibration_id=getattr(config, 'CALIBRATION_ID', None),
                        fps_target=float(getattr(config, 'FPS', 30.0)),
                        fps_achieved=None,
                        filter_min_cutoff=float(getattr(config, 'FILTER_MIN_CUTOFF', 0.0)),
                        filter_beta=float(getattr(config, 'FILTER_BETA', 0.0)),
                        filter_d_cutoff=float(getattr(config, 'FILTER_D_CUTOFF', 0.0)),
                        metadata={
                            'recording_mode': recording_mode,
                            'dynamic_sync_threshold_enabled': bool(getattr(config, 'SYNC_DYNAMIC_THRESHOLD_ENABLED', False)),
                            'sync_dynamic_factor': float(getattr(config, 'SYNC_DYNAMIC_FACTOR', 0.0)),
                        },
                    )
                self.is_recording = True
                
                # Reset Tree
                for item in self.tree.get_children():
                    self.tree.delete(item)
                    
                self.status_label.config(text="● RECORDING", fg='#ff0055') # VS2 Red
                self.record_btn.config(text="⏹ Stop Capture", fg='#ff0055')
            except Exception as e:
                messagebox.showerror("Error", f"Failed to start recording: {e}")
        else:
            if hasattr(self.db, 'save_processing_metadata') and self.session_id:
                fps_now = None
                try:
                    fps_now = float(self.visualizer.get_fps())
                except Exception:
                    fps_now = None
                sync_threshold_ms = float(config.SYNC_TIME_THRESHOLD_MS)
                if self.coordinator and hasattr(self.coordinator, 'get_current_sync_threshold_ms'):
                    sync_threshold_ms = float(self.coordinator.get_current_sync_threshold_ms())
                self.db.save_processing_metadata(
                    sync_threshold_ms=sync_threshold_ms,
                    sync_strategy='timestamp_match',
                    clock_sync_enabled=bool(getattr(config, 'ENABLE_CLOCK_SYNC', False)),
                    clock_sync_interval_sec=float(getattr(config, 'CLOCK_SYNC_INTERVAL_SEC', 0.0)),
                    triangulation_method='opencv_triangulate',
                    triangulation_version='1.0',
                    smoothing_version='1.0',
                    calibration_id=getattr(config, 'CALIBRATION_ID', None),
                    fps_target=float(getattr(config, 'FPS', 30.0)),
                    fps_achieved=fps_now,
                    filter_min_cutoff=float(getattr(config, 'FILTER_MIN_CUTOFF', 0.0)),
                    filter_beta=float(getattr(config, 'FILTER_BETA', 0.0)),
                    filter_d_cutoff=float(getattr(config, 'FILTER_D_CUTOFF', 0.0)),
                    metadata={
                        'recording_mode': 'stereo' if MULTI_CAMERA_MODE == 'master' else 'single',
                        'dynamic_sync_threshold_enabled': bool(getattr(config, 'SYNC_DYNAMIC_THRESHOLD_ENABLED', False)),
                        'sync_dynamic_factor': float(getattr(config, 'SYNC_DYNAMIC_FACTOR', 0.0)),
                    },
                    session_id=self.session_id,
                )
            self.db.stop_recording()
            self.is_recording = False
            self.status_label.config(text="● READY", fg='#00ff88')
            self.record_btn.config(text="▶ Start Capture", fg='#00ff88')
            
            messagebox.showinfo("Session Complete", 
                              f"Recording saved.\nSession ID: {self.session_id[:8]}...")

    def update_table(self, data):
        """Update the table with new frame data (VS2 style live feed)."""
        pass # Not used directly, using update_table_safe

    def download_data(self):
        csv_content = self.db.export_latest_session_csv()
        if not csv_content:
            messagebox.showwarning("No Data", "No recordings found to export")
            return
        
        filename = f"mocap_{int(time.time())}.csv"
        with open(filename, 'w') as f:
            f.write(csv_content)
        messagebox.showinfo("Export Complete", f"Dataset saved to {filename}")

    def _put_latest(self, q: queue.Queue, item) -> None:
        """Enqueue item onto a maxsize=1 queue, dropping the oldest if full.
        This enforces latest-only semantics: stale work/frames are never kept.
        """
        try:
            q.put_nowait(item)
        except queue.Full:
            try:
                q.get_nowait()
            except queue.Empty:
                pass
            try:
                q.put_nowait(item)
            except queue.Full:
                pass

    def _send_worker(self):
        """Single background thread that drains the network send queue.
        Replaces the per-frame threading.Thread spawn which caused thread buildup."""
        while self.running:
            try:
                item = self._send_queue.get(timeout=0.5)
                if item is None:
                    break
                frame_number, timestamp, results, frame_copy = item
                self.network_server.send_frame_data(frame_number, timestamp, results, frame_copy)
            except queue.Empty:
                continue
            except Exception as e:
                print(f"[SERVER] Send worker error: {e}")

    def _remote_decode_worker(self):
        """Background worker that decodes remote JPEG frames without blocking video_loop."""
        while self.running:
            try:
                item = self._remote_decode_queue.get(timeout=0.5)
                if item is None:
                    break

                jpeg, display_width, display_height = item
                if not jpeg:
                    continue

                jpg_np = np.frombuffer(bytes(jpeg), dtype=np.uint8)
                decoded = cv2.imdecode(jpg_np, cv2.IMREAD_COLOR)
                if decoded is not None:
                    remote_frame_gpu = cv2.resize(decoded, (display_width, display_height))
                    self.remote_frame = remote_frame_gpu
                    self._last_remote_gpu = remote_frame_gpu
            except queue.Empty:
                continue
            except Exception as e:
                if self.frame_count % 100 == 0:
                    print(f"[Display] Remote decode error: {e}")

    def _db_save_worker(self):
        """Background worker: drain the async DB save queue so SQLite writes never block the pipeline."""
        while True:
            try:
                item = self._db_save_queue.get(timeout=0.5)
            except queue.Empty:
                if not self.running:
                    break
                continue
            if item is None:
                break
            try:
                op = item[0]
                if op == 'synced':
                    _, ts, pc1, pc2, pose_3d, synced_batch = item
                    self._save_trace['db_dequeue_synced'] += 1
                    if self._save_trace_enabled and self._save_trace['db_dequeue_synced'] % self._save_trace_every == 0:
                        p3d_n = 0
                        kin_n = 0
                        pc1_ok = self._has_pose_payload(pc1)
                        pc2_data = self._extract_pc2_pose_data(pc2)
                        pc2_ok = len(pc2_data) > 0
                        if isinstance(pose_3d, dict):
                            p3d_n = len(pose_3d.get('pose_3d', {}) or {})
                            kin = pose_3d.get('kinematics_3d', {}) or {}
                            if isinstance(kin, dict):
                                kin_n = len(kin.get('flat_export', {}) or {})
                        print(
                            f"[SaveTrace][DBDequeue][synced] n={self._save_trace['db_dequeue_synced']} "
                            f"batch={len(synced_batch) if synced_batch else 0} "
                            f"pc1={'Y' if pc1_ok else 'N'} pc2={'Y' if pc2_ok else 'N'} "
                            f"pose3d_pts={p3d_n} kin_keys={kin_n} q={self._db_save_queue.qsize()}"
                        )
                    self.db.save_synced_frame(ts, pc1, pc2, pose_3d)
                    if self.coordinator and hasattr(self.coordinator, 'save_frame_to_database'):
                        self.coordinator.save_frame_to_database(synced_batch, pose_3d)
                elif op == 'mono':
                    _, ts, local_res, windows_res, mono_pose_3d = item
                    self._save_trace['db_dequeue_mono'] += 1
                    if self._save_trace_enabled and self._save_trace['db_dequeue_mono'] % self._save_trace_every == 0:
                        p3d_n = 0
                        if isinstance(mono_pose_3d, dict):
                            p3d_n = len(mono_pose_3d.get('pose_3d', {}) or {})
                        print(
                            f"[SaveTrace][DBDequeue][mono] n={self._save_trace['db_dequeue_mono']} "
                            f"local={'Y' if bool(local_res) else 'N'} windows={'Y' if bool(windows_res) else 'N'} "
                            f"pose3d_pts={p3d_n} q={self._db_save_queue.qsize()}"
                        )
                    self.db.save_synced_frame(ts, local_res, windows_res, mono_pose_3d)
            except Exception as e:
                if self.running:
                    print(f"[DBSave] Write error: {e}")

    def _master_compute_worker(self):
        """Background worker: run sync/3D compute while capture thread keeps saving next frames."""
        while self.running:
            try:
                item = self._master_compute_queue.get(timeout=0.5)
                if item is None:
                    break

                remote_camera_id, frame_count, t_frame_start = item

                t_sync_start = time.perf_counter()
                synced_batch = self.coordinator.get_synchronized_batch() if self.coordinator else None
                t_sync_end = time.perf_counter()
                if self.coordinator:
                    self.coordinator.record_latency('sync', (t_sync_end - t_sync_start) * 1000)

                pose_3d = None
                pc1_res = None
                pc2_res = None
                if synced_batch and len(synced_batch) >= 2:
                    pc1_res = next((f.results for f in synced_batch if f.camera_id == 'local_cam'), None)
                    remote_frames = [f for f in synced_batch if f.camera_id != 'local_cam']
                    if remote_frames:
                        chosen_remote = next((f for f in remote_frames if f.camera_id == remote_camera_id), remote_frames[0])
                        remote_camera_id = chosen_remote.camera_id
                        pc2_res = chosen_remote.results

                    t_tri_start = time.perf_counter()
                    pose_3d = self.coordinator.get_synced_3d_pose(synced_batch) if self.coordinator else None
                    t_tri_end = time.perf_counter()
                    if self.coordinator:
                        self.coordinator.record_latency('triangulation', (t_tri_end - t_tri_start) * 1000)

                if self.coordinator and t_frame_start is not None:
                    t_total = (time.perf_counter() - t_frame_start) * 1000
                    self.coordinator.record_latency('total', t_total)

                payload = {
                    'frame_count': frame_count,
                    'remote_camera_id': remote_camera_id,
                    'synced_batch': synced_batch,
                    'pc1_res': pc1_res,
                    'pc2_res': pc2_res,
                    'pose_3d': pose_3d,
                }
                self._put_latest(self._master_result_queue, payload)
            except queue.Empty:
                continue
            except Exception as e:
                if self.running:
                    print(f"[MasterCompute] Worker error: {e}")

    def _drain_master_results(self):
        """Apply compute-worker outputs to GUI/state without blocking capture loop."""
        while True:
            try:
                out = self._master_result_queue.get_nowait()
            except queue.Empty:
                break

            frame_count = int(out.get('frame_count', self.frame_count))
            remote_camera_id = out.get('remote_camera_id')
            synced_batch = out.get('synced_batch')
            pc1_res = out.get('pc1_res')
            pc2_res = out.get('pc2_res')
            pose_3d = out.get('pose_3d')

            # ── Remote metrics: update from latest remote buffer frame even without sync ──
            if remote_camera_id and not pc2_res and self.coordinator:
                buf = self.coordinator.frame_buffers.get(remote_camera_id)
                if buf and len(buf) > 0:
                    pc2_res = buf[-1].results
            if not pc2_res and self.coordinator:
                freshest = None
                freshest_cam = None
                for cam_id, buf in self.coordinator.frame_buffers.items():
                    if cam_id == 'local_cam' or len(buf) == 0:
                        continue
                    candidate = buf[-1]
                    if freshest is None or candidate.received_at > freshest.received_at:
                        freshest = candidate
                        freshest_cam = cam_id
                if freshest is not None:
                    pc2_res = freshest.results
                    remote_camera_id = freshest_cam

            pc2_has_pose = self._has_pose_payload(pc2_res)
            remote_metrics_new = self._compute_stream_metrics(remote_camera_id, pc2_res) \
                if (pc2_has_pose and remote_camera_id) else None
            if remote_metrics_new:
                self.latest_remote_metrics = remote_metrics_new
                # Push updated remote metrics to GUI (coalesced)
                self._schedule_metrics_gui({
                    'local': self.latest_local_metrics,
                    'remote': self.latest_remote_metrics
                })
                if frame_count % 5 == 0:
                    self._enqueue_ui_task(self.update_table_safe, {
                        'local': self.latest_local_metrics,
                        'remote': self.latest_remote_metrics
                    })

            if synced_batch and len(synced_batch) >= 2:
                self._master_last_sync_batch = synced_batch

                local_metrics = self._compute_stream_metrics('local_cam', pc1_res) if pc1_res else None
                if local_metrics:
                    self.latest_local_metrics = local_metrics

                if frame_count % 5 == 0:
                    self._enqueue_ui_task(self.update_table_safe, {
                        'local': self.latest_local_metrics,
                        'remote': self.latest_remote_metrics
                    })

                if self.is_recording:
                    # Normalize wrapped remote payload so DB serializer always
                    # receives compact landmark keys at top level.
                    if isinstance(pc2_res, dict):
                        inner = pc2_res.get('results')
                        if isinstance(inner, dict):
                            normalized = dict(pc2_res)
                            if 'landmarks' not in normalized and inner.get('landmarks') is not None:
                                normalized['landmarks'] = inner.get('landmarks')
                            if 'packet_landmarks' not in normalized and inner.get('packet_landmarks') is not None:
                                normalized['packet_landmarks'] = inner.get('packet_landmarks')
                            if 'pose_landmarks' not in normalized and inner.get('pose_landmarks') is not None:
                                normalized['pose_landmarks'] = inner.get('pose_landmarks')
                            pc2_res = normalized

                    self._save_trace['prequeue_synced'] += 1
                    if self._save_trace_enabled and self._save_trace['prequeue_synced'] % self._save_trace_every == 0:
                        p3d_n = 0
                        kin_n = 0
                        pc1_ok = self._has_pose_payload(pc1_res)
                        pc2_data = self._extract_pc2_pose_data(pc2_res)
                        pc2_ok = len(pc2_data) > 0
                        if isinstance(pose_3d, dict):
                            p3d_n = len(pose_3d.get('pose_3d', {}) or {})
                            kin = pose_3d.get('kinematics_3d', {}) or {}
                            if isinstance(kin, dict):
                                kin_n = len(kin.get('flat_export', {}) or {})
                        print(
                            f"[SaveTrace][PreQueue][synced] n={self._save_trace['prequeue_synced']} "
                            f"remote_id={remote_camera_id} batch={len(synced_batch)} "
                            f"pc1={'Y' if pc1_ok else 'N'} pc2={'Y' if pc2_ok else 'N'} "
                            f"pose3d_pts={p3d_n} kin_keys={kin_n}"
                        )
                    self._db_save_queue.put_nowait(
                        ('synced', time.time(), pc1_res, pc2_res, pose_3d, synced_batch)
                    )

            elif self.is_recording:
                # No stereo sync (remote camera disconnected/unavailable).
                # Extract local frame directly from the batch even if len < 2.
                local_res = pc1_res  # already set when len >= 2 (rare overlap)
                if local_res is None and synced_batch:
                    local_res = next(
                        (f.results for f in synced_batch if f.camera_id == 'local_cam'), None
                    )
                if local_res:
                    # Compute monocular pose_3d from the single local frame
                    mono_pose_3d = None
                    if synced_batch and self.coordinator:
                        local_batch = [f for f in synced_batch if f.camera_id == 'local_cam']
                        if local_batch:
                            try:
                                mono_pose_3d = self.coordinator.get_synced_3d_pose(local_batch)
                            except Exception:
                                pass
                    # No stereo sync — save local camera frame alone.
                    # Also check if the Windows buffer has a recent frame: if so, save it
                    # as pc2 so Windows data is never silently discarded even when sync fails.
                    windows_res = None
                    if self.coordinator:
                        win_candidate = None
                        for cam_id, buf in self.coordinator.frame_buffers.items():
                            if cam_id == 'local_cam' or len(buf) == 0:
                                continue
                            candidate = buf[-1]
                            if win_candidate is None or candidate.received_at > win_candidate.received_at:
                                win_candidate = candidate
                        if win_candidate is not None:
                            windows_res = win_candidate.results  # latest non-local frame
                    self._save_trace['prequeue_mono'] += 1
                    if self._save_trace_enabled and self._save_trace['prequeue_mono'] % self._save_trace_every == 0:
                        p3d_n = 0
                        if isinstance(mono_pose_3d, dict):
                            p3d_n = len(mono_pose_3d.get('pose_3d', {}) or {})
                        print(
                            f"[SaveTrace][PreQueue][mono] n={self._save_trace['prequeue_mono']} "
                            f"remote_id={remote_camera_id} local={'Y' if bool(local_res) else 'N'} "
                            f"windows={'Y' if bool(windows_res) else 'N'} pose3d_pts={p3d_n}"
                        )
                    self._db_save_queue.put_nowait(
                        ('mono', time.time(), local_res, windows_res, mono_pose_3d)
                    )

                if pose_3d:
                    self.latest_quality = self._extract_quality_metrics(pose_3d)
                    if frame_count % 100 == 0:
                        print(f"✅ 3D Pose Computed! {len(pose_3d.get('pose_3d',[]))} landmarks")
                        if self.coordinator:
                            self.coordinator._log_latency_summary()

    def _extract_metric_inputs(self, results):
        """Return (pose_lm, face_lm) as list[dict] from local or remote result formats."""
        if not isinstance(results, dict):
            return [], []

        pose_lm = []
        face_lm = []

        pose_obj = results.get('pose')
        if pose_obj and hasattr(pose_obj, 'pose_landmarks') and pose_obj.pose_landmarks:
            pose_lm = [
                {'x': lm.x, 'y': lm.y, 'z': lm.z, 'v': getattr(lm, 'visibility', 1.0)}
                for lm in pose_obj.pose_landmarks[0]
            ]
        else:
            pose_ser = results.get('pose_landmarks')
            if isinstance(pose_ser, list) and len(pose_ser) > 0:
                first = pose_ser[0]
                src = first if isinstance(first, list) else pose_ser
                for lm in src:
                    if isinstance(lm, dict):
                        pose_lm.append({
                            'x': float(lm.get('x', 0.0)),
                            'y': float(lm.get('y', 0.0)),
                            'z': float(lm.get('z', 0.0)),
                            'v': float(lm.get('visibility', lm.get('v', 1.0)))
                        })

        if not pose_lm:
            packet = results.get('packet_landmarks')
            if packet is None:
                packet = results.get('landmarks')
            if isinstance(packet, list):
                src = packet[0] if packet and isinstance(packet[0], list) else packet
                for lm in src:
                    if isinstance(lm, dict):
                        pose_lm.append({
                            'x': float(lm.get('x', 0.0)),
                            'y': float(lm.get('y', 0.0)),
                            'z': float(lm.get('z', 0.0)),
                            'v': float(lm.get('conf', lm.get('visibility', 1.0)))
                        })

        if not pose_lm and isinstance(results.get('results'), dict):
            inner_pose, _ = self._extract_metric_inputs(results.get('results'))
            if inner_pose:
                pose_lm = inner_pose

        face_obj = results.get('face')
        if face_obj and hasattr(face_obj, 'face_landmarks') and face_obj.face_landmarks:
            face_lm = [{'x': lm.x, 'y': lm.y, 'z': lm.z} for lm in face_obj.face_landmarks[0]]
        else:
            face_ser = results.get('face_landmarks')
            if isinstance(face_ser, list) and len(face_ser) > 0:
                first = face_ser[0]
                src = first if isinstance(first, list) else face_ser
                for lm in src:
                    if isinstance(lm, dict):
                        face_lm.append({
                            'x': float(lm.get('x', 0.0)),
                            'y': float(lm.get('y', 0.0)),
                            'z': float(lm.get('z', 0.0))
                        })

        return pose_lm, face_lm

    def _extract_pc2_pose_data(self, pc2_res):
        """Extract PC2 pose payload using fallback keys and normalize to list-of-people."""
        if not isinstance(pc2_res, dict):
            return []

        # 1. Try to get the data from any of the possible keys
        raw_pc2_data = pc2_res.get('pose_landmarks') or pc2_res.get('landmarks') or pc2_res.get('packet_landmarks') or []

        # 2. Fix list nesting for compact flat payload
        if raw_pc2_data and isinstance(raw_pc2_data, list) and len(raw_pc2_data) > 0 and isinstance(raw_pc2_data[0], dict):
            pc2_data = [raw_pc2_data]
        else:
            pc2_data = raw_pc2_data

        return pc2_data if isinstance(pc2_data, list) else []

    def _has_pose_payload(self, results) -> bool:
        """True when results contain at least one pose payload in any supported key format."""
        if not isinstance(results, dict):
            return False

        pose_lm, _ = self._extract_metric_inputs(results)
        if pose_lm:
            return True

        # Support nested transport wrappers defensively.
        inner = results.get('results')
        if isinstance(inner, dict):
            pose_lm, _ = self._extract_metric_inputs(inner)
            return bool(pose_lm)

        return False

    def _compute_stream_metrics(self, camera_key, results):
        """Compute smoothed metrics + kinematics for a specific camera stream."""
        pose_lm, face_lm = self._extract_metric_inputs(results)
        if not pose_lm:
            return None

        body_metrics = Calculations.get_body_metrics(pose_lm)
        face_metrics = Calculations.get_face_metrics(face_lm) if face_lm else {}
        raw_metrics = {**body_metrics, **face_metrics}
        norm_metrics = Calculations.normalize_metrics(raw_metrics, pose_lm)

        state = self._metric_state.setdefault(camera_key, {'prev_lm': [], 'prev_metrics': {}, 'prev_time': None})
        metrics = Calculations.filter_and_smooth(norm_metrics, state['prev_metrics'])

        now = time.time()
        if state['prev_time'] is not None:
            dt = now - state['prev_time']
            if dt > 0:
                kinematics = Calculations.get_kinematics(
                    pose_lm, state['prev_lm'], metrics, state['prev_metrics'], dt
                )
                metrics.update(kinematics)

        state['prev_lm'] = pose_lm
        state['prev_metrics'] = metrics
        state['prev_time'] = now

        if camera_key == 'local_cam':
            self.prev_lm = pose_lm
            self.prev_metrics = metrics
            self.prev_time = now

        return metrics

    def video_loop(self):
        while self.running:
            t_frame_start = time.perf_counter()
            
            # ── Capture ──
            t_cap_start = time.perf_counter()
            self.camera.wait_for_frame(timeout=0.033)
            frame = self.camera.read()
            if frame is None:
                continue
            t_cap_end = time.perf_counter()
            
            # 1. Mirroring (Horizontal Flip)
            if self.mirror_active:
                frame = cv2.flip(frame, 1)
            
            # ── Detection ──
            t_det_start = time.perf_counter()
            results = self.detector.process(frame)
            t_det_end = time.perf_counter()
            
            # Track latency in coordinator if available
            if self.coordinator:
                self.coordinator.record_latency('capture', (t_cap_end - t_cap_start) * 1000)
                self.coordinator.record_latency('detection', (t_det_end - t_det_start) * 1000)
            
            # --- PHYSICS CORRECTION ---
            # Corrects bone lengths and smooths jitter
            results = self.corrector.process(results)
            # --------------------------
            
            # --- CALCULATE METRICS ---
            if self.frame_count % 2 == 0:
                try:
                    local_metrics = self._compute_stream_metrics('local_cam', results)
                    if local_metrics:
                        self.latest_local_metrics = local_metrics
                        # Always push local metrics to GUI; remote metrics fill in when sync succeeds
                        self._schedule_metrics_gui({
                            'local': self.latest_local_metrics,
                            'remote': self.latest_remote_metrics
                        })
                except:
                    pass
            # -------------------------
            
            # --- DRAW VISUALIZATION FIRST (for network transmission) ---
            if self.markers_active and results:
                frame = self.visualizer.draw_landmarks(frame, results)
            # -----------------------------------------------------------
            
            # --- NETWORK BROADCASTING (Server Mode) - via send queue ---
            if self.network_server:
                timestamp = int(time.time() * 1e9)
                self._put_latest(self._send_queue, (self.frame_count, timestamp, results, frame.copy()))

                # Debug only first 3 frames
                if self.frame_count <= 3:
                    print(f"[SERVER] Queued frame {self.frame_count}")
            # -----------------------------------------
            
            # --- RECEIVE REMOTE CAMERA (Master Mode) ---
            if self.coordinator:
                # Master mode: Add our LOCAL camera to sync buffer too!
                # The coordinator receives PC2's frames automatically,
                # but we need to add PC1's local frames manually
                # Use wall-clock time (epoch nanoseconds) for cross-machine sync
                timestamp = int(time.time() * 1e9)
                
                # Create a local frame data entry
                sync_results = results
                if hasattr(self.coordinator, 'compact_results_for_sync'):
                    sync_results = self.coordinator.compact_results_for_sync(results, include_jpeg=False)
                local_frame_data = FrameData(
                    camera_id='local_cam',
                    frame_number=self.frame_count,
                    timestamp=timestamp,
                    results=sync_results,
                    received_at=time.time()  # Mac wall-clock seconds — same units as remote cam received_at
                )
                
                # Add to coordinator's buffer manually
                if hasattr(self.coordinator, 'frame_buffers'):
                    # Use frame_buffers (plural) - it's a dict of deques
                    self.coordinator.frame_buffers['local_cam'].append(local_frame_data)
                    
                    # Debug only on errors
                    pass
                else:
                    if self.frame_count <= 3:
                        print(f"[MASTER ERROR] Coordinator has no frame_buffers attribute!")
            # -----------------------------------------
            
            if self.is_recording:
                # Save first - BUT ONLY if not in master mode (master saves synced batches)
                if MULTI_CAMERA_MODE != 'master':
                    self.db.save_frame(results)
                
                # Update GUI safely (Throttled)
                if self.frame_count % 5 == 0:
                     self._enqueue_ui_task(self.update_table_safe, {'local': self.latest_local_metrics, 'remote': {}})
                     
            self.frame_count += 1

            # Draw FPS overlay (landmarks already drawn above before network send)
            frame = self.visualizer.draw_fps(frame)
            fps = self.visualizer.get_fps()
            
            # Update FPS label
            try:
                self._enqueue_ui_task(self.fps_label.config, text=f"FPS: {fps:04.1f}")
            except: pass 
            
            # --- DUAL CAMERA DISPLAY (Master Mode) ---
            # Display uses CPU only — MediaPipe already uses Metal for inference.
            # GPU tensor path was causing 3GB/s MPS memory leak (never freed per-frame tensors).
            if MULTI_CAMERA_MODE == 'master' and self.coordinator:
                sync_label = "WAITING"
                # Prefer the freshest non-local camera buffer. This avoids hard-coding
                # remote IDs (e.g., cam_0 vs cam_1) and keeps pc2_res non-empty.
                remote_camera_id = None
                remote_candidates = [
                    (cid, buf[-1].received_at)
                    for cid, buf in self.coordinator.frame_buffers.items()
                    if cid != 'local_cam' and len(buf) > 0
                ]
                if remote_candidates:
                    remote_camera_id = max(remote_candidates, key=lambda x: x[1])[0]
                else:
                    remote_camera_id = next(
                        (cid for cid in self.coordinator.frame_buffers.keys() if cid != 'local_cam'),
                        'cam_0'
                    )

                # ── Throttled visual display (~10 fps) — build/send every 3rd frame only ──
                if ABLATION_DISABLE_DISPLAY_THROTTLE or self.frame_count % 3 == 0:
                    display_width, display_height = 640, 360  # 16:9 native; 4× less memory than original 1280×720

                    # 1. Local frame — CPU resize
                    local_display = cv2.resize(frame, (display_width, display_height))

                    # 2. Remote frame — CPU JPEG decode + resize
                    remote_display = np.zeros((display_height, display_width, 3), dtype=np.uint8)
                    if remote_camera_id in self.coordinator.frame_buffers:
                        buf = self.coordinator.frame_buffers[remote_camera_id]
                        if len(buf) > 0:
                            latest = buf[-1]
                            jpeg = None
                            if hasattr(self.coordinator, 'get_latest_camera_jpeg'):
                                jpeg = self.coordinator.get_latest_camera_jpeg(remote_camera_id)
                            if not jpeg:
                                jpeg = latest.results.get('frame_jpeg')
                            if jpeg:
                                try:
                                    self._remote_decode_queue.put_nowait((jpeg, display_width, display_height))
                                except queue.Full:
                                    try:
                                        self._remote_decode_queue.get_nowait()
                                    except queue.Empty:
                                        pass
                                    try:
                                        self._remote_decode_queue.put_nowait((jpeg, display_width, display_height))
                                    except queue.Full:
                                        pass

                            if self.remote_frame is not None:
                                remote_display = self.remote_frame
                                sync_label = "LIVE" if jpeg else "BUFFERED"
                                self._last_remote_frame_time = time.time()  # mark last live frame
                        else:
                            if self._last_remote_gpu is not None:
                                remote_display = self._last_remote_gpu
                                sync_label = "BUFFERED"
                                self._last_remote_frame_time = time.time()
                        # Debug: print every 5s when no frames arriving
                        if self.frame_count % 150 == 0 and sync_label == "WAITING":
                            total = sum(self.coordinator.stats['frames_received'].values())
                            print(f"[Display] {remote_camera_id} buf size={len(buf)}, total_rx={total} — check Windows firewall (ports 6000,6001)")

                    # 3. Cross-camera occlusion overlay: draw landmarks from the remote
                    #    camera on the local frame when that landmark is occluded locally.
                    if len(self.coordinator.frame_buffers.get(remote_camera_id, [])) > 0:
                        remote_latest = self.coordinator.frame_buffers[remote_camera_id][-1]
                        remote_lm_list = None
                        r_res = remote_latest.results if remote_latest else {}
                        if isinstance(r_res, dict):
                            remote_lm_list = (
                                r_res.get('pose_landmarks')
                                or r_res.get('packet_landmarks')
                                or r_res.get('landmarks')
                            )
                        self._draw_cross_camera_landmarks(local_display, results, remote_lm_list)

                    # 4. Combine side-by-side and push to tkinter window
                    # Promote WAITING → DISCONNECTED after 5 s of silence
                    remote_label_color = (0, 255, 255)  # cyan = normal
                    if sync_label == "WAITING" and self._last_remote_frame_time is not None:
                        if time.time() - self._last_remote_frame_time > 5.0:
                            sync_label = "DISCONNECTED"
                            remote_label_color = (0, 0, 255)  # red = lost connection
                    cv2.putText(local_display, "LOCAL-MAC", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(remote_display, "REMOTE-WIN", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, remote_label_color, 2)
                    combined = np.hstack([local_display, remote_display])
                    self._schedule_display_tkinter(combined, "Dual Camera — Master")
                    del combined, local_display, remote_display  # explicit free — 320×240×3×2 arrays

                # Periodic GC to reclaim MediaPipe/Metal objects
                if self.frame_count % 150 == 0:
                    gc.collect()

                # --- PIPELINED SYNC + 3D COMPUTE ---
                # Save/capture continues in this thread while worker computes previous batch.
                self._put_latest(self._master_compute_queue, (remote_camera_id, self.frame_count, t_frame_start))

                # Drain completed compute results (non-blocking)
                self._drain_master_results()

                synced_batch = self._master_last_sync_batch

                # DEBUG SYNC FAILURE (Conditional print)
                if not synced_batch and self.frame_count % 30 == 0:
                    if remote_camera_id in self.coordinator.frame_buffers and 'local_cam' in self.coordinator.frame_buffers:
                        c0_buf = self.coordinator.frame_buffers[remote_camera_id]
                        lc_buf = self.coordinator.frame_buffers['local_cam']
                        if len(c0_buf) > 0 and len(lc_buf) > 0:
                            t_remote = c0_buf[-1].timestamp
                            t_local = lc_buf[-1].timestamp
                            diff_ms = abs(t_remote - t_local) / 1e6
                            threshold_ms = float(config.SYNC_TIME_THRESHOLD_MS)
                            if self.coordinator and hasattr(self.coordinator, 'get_current_sync_threshold_ms'):
                                threshold_ms = float(self.coordinator.get_current_sync_threshold_ms())
                            print(f"[Sync Debug] Latest Frame Delta: {diff_ms:.1f}ms (Threshold: {threshold_ms:.1f}ms)")
                elif synced_batch and len(synced_batch) >= 2:
                    sync_label = "SYNCED ✓"

                # Raw frame archival remains on capture thread
                if self.is_recording and config.SAVE_RAW_FRAMES:
                    self.db.save_raw_frame(frame, self.frame_count, 'local_cam')
                    if self.remote_frame is not None:
                        self.db.save_raw_frame(self.remote_frame, self.frame_count, remote_camera_id)
                
                # Update latency panel every ~1 second
                if self.frame_count % 30 == 0:
                    self._enqueue_ui_task(self._update_latency_panel)
                    self._enqueue_ui_task(self._update_quality_panel)
                    self._enqueue_ui_task(self._update_clock_sync_panel)

                if self.frame_count % 300 == 0 and hasattr(self.coordinator, 'get_memory_status'):
                    try:
                        mem = self.coordinator.get_memory_status()
                        rss = mem.get('rss_mb')
                        buf_sizes = mem.get('frame_buffer_sizes', {})
                        _peak_note = " (peak/maxrss)" if platform.system() == 'Darwin' else ""
                        print(f"[Memory] RSS={rss:.1f}MB{_peak_note} buffers={buf_sizes} db_q={self.db.queue.qsize() if hasattr(self.db, 'queue') else 'n/a'} ui_q={self._ui_task_queue.qsize()}")
                    except Exception:
                        pass

                # ── Ablation study RSS telemetry ──
                # Shows current RSS and which memory fixes are ACTIVE.
                # To ablate a fix: set its ABLATION_DISABLE_* flag to True in config.py,
                # restart, and compare RSS here to measure that fix's contribution.
                if ABLATION_RSS_LOG_INTERVAL > 0 and self.frame_count % ABLATION_RSS_LOG_INTERVAL == 0:
                    rss_now = _get_rss_mb()
                    active = []
                    if not ABLATION_DISABLE_FRAME_RESIZE:     active.append('frame_resize')
                    if not ABLATION_DISABLE_DISPLAY_THROTTLE: active.append('display_throttle')
                    if not ABLATION_DISABLE_PHOTO_REUSE:      active.append('photo_reuse')
                    if not ABLATION_DISABLE_CLAHE_DEL:        active.append('clahe_del')
                    _peak_note = " (peak/maxrss — install psutil for current)" if _RSS_IS_PEAK else ""
                    print(f"[ABLATION] frame={self.frame_count} RSS={rss_now:.1f}MB{_peak_note}  active_fixes={active}")

                # FINAL GPU DISPLAY (or CPU Fallback defined earlier)
                # Note: Labels are already applied in the GPU/CPU blocks above
            else:
                # Single camera or server mode — show in tkinter on Mac, cv2 on Windows
                if platform.system() == 'Darwin':
                    _frame_copy = frame.copy()
                    self._schedule_display_tkinter(_frame_copy, "MoCap Live Feed")
                else:
                    cv2.imshow("MoCap Live Feed", frame)

            # cv2.waitKey crashes on macOS when called from a background thread
            if platform.system() != 'Darwin':
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.running = False
                    break

            # Explicitly release per-frame heavy objects to keep memory flat.
            try:
                del local_frame_data
            except Exception:
                pass
            try:
                del sync_results
            except Exception:
                pass
            try:
                del results
            except Exception:
                pass
            try:
                del frame
            except Exception:
                pass

            # Throttled forced GC to avoid lazy collector drift in long sessions.
            if self.frame_count % 100 == 0:
                gc.collect()
        
        self.cleanup()

    # ── MediaPipe pose connections (subset: body skeleton) ──
    _POSE_CONNECTIONS = [
        (11, 12), (11, 13), (13, 15), (12, 14), (14, 16),  # arms
        (11, 23), (12, 24), (23, 24),                        # torso
        (23, 25), (25, 27), (24, 26), (26, 28),             # legs
        (15, 17), (15, 19), (15, 21),                        # left hand
        (16, 18), (16, 20), (16, 22),                        # right hand
        (27, 29), (27, 31), (28, 30), (28, 32),              # feet
    ]

    def _draw_cross_camera_landmarks(self, local_display, local_results, remote_lm_list):
        """
        Overlay remote-camera landmarks (orange) on the local frame for joints that
        are occluded / low-confidence locally.  Uses normalised [0..1] coords so no
        calibration required — purely for visual awareness.
        """
        if not remote_lm_list:
            return

        h, w = local_display.shape[:2]

        # ── Parse remote landmarks (serialised list-of-dicts) ──
        remote_pts = {}
        src = remote_lm_list
        if isinstance(src, list) and len(src) > 0 and isinstance(src[0], list):
            src = src[0]  # unwrap nested list
        for idx, lm in enumerate(src):
            if isinstance(lm, dict):
                remote_pts[idx] = {
                    'x': float(lm.get('x', 0.0)),
                    'y': float(lm.get('y', 0.0)),
                    'v': float(lm.get('visibility', lm.get('v', 1.0)))
                }

        if not remote_pts:
            return

        # ── Gather local visibility per landmark ──
        local_vis = {}
        pose_obj = local_results.get('pose') if isinstance(local_results, dict) else None
        if pose_obj and hasattr(pose_obj, 'pose_landmarks') and pose_obj.pose_landmarks:
            for idx, lm in enumerate(pose_obj.pose_landmarks[0]):
                local_vis[idx] = getattr(lm, 'visibility', 1.0)

        # ── Draw remote skeleton on local display ──
        # Connections: only draw if remote landmark is well-visible AND locally occluded/absent
        OCCLUSION_THRESH = 0.35  # local visibility below this → show remote overlay
        REMOTE_VIS_MIN   = 0.40  # remote must be at least this confident

        def _pt(idx):
            lm = remote_pts.get(idx)
            if lm is None or lm['v'] < REMOTE_VIS_MIN:
                return None
            # Use remote for this joint only if locally occluded or missing
            loc_v = local_vis.get(idx, 0.0)
            if loc_v > OCCLUSION_THRESH:
                return None  # locally visible — no need to show remote
            px = int(lm['x'] * w)
            py = int(lm['y'] * h)
            if 0 <= px < w and 0 <= py < h:
                return (px, py)
            return None

        drawn_nodes = set()
        for (a, b) in self._POSE_CONNECTIONS:
            pa = _pt(a)
            pb = _pt(b)
            if pa is not None and pb is not None:
                cv2.line(local_display, pa, pb, (0, 165, 255), 2, cv2.LINE_AA)  # orange
                drawn_nodes.add(a)
                drawn_nodes.add(b)
            elif pa is not None:
                drawn_nodes.add(a)
            elif pb is not None:
                drawn_nodes.add(b)

        for idx in drawn_nodes:
            lm = remote_pts.get(idx)
            if lm is None:
                continue
            px = int(lm['x'] * w)
            py = int(lm['y'] * h)
            if 0 <= px < w and 0 <= py < h:
                cv2.circle(local_display, (px, py), 5, (0, 165, 255), -1, cv2.LINE_AA)

        # ── Legend ──
        if drawn_nodes:
            cv2.putText(local_display, "WIN overlay", (10, h - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 165, 255), 1, cv2.LINE_AA)

    def update_table_safe(self, data):
        # Update Table with Key Angles (Matches Headers)
        local_metrics = {}
        remote_metrics = {}
        if isinstance(data, dict):
            local_metrics = data.get('local') or {}
            remote_metrics = data.get('remote') or {}

        keys = [
            "Angle_Shoulder_L", "Angle_Shoulder_R",
            "Angle_Elbow_L", "Angle_Elbow_R",
            "Angle_Knee_L", "Angle_Knee_R",
            "Velocity_Angle_Elbow_R", "Velocity_Wrist_R"
        ]

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
            _insert_row("MAC", local_metrics)
        if remote_metrics:
            _insert_row("WIN", remote_metrics)
        
        children = self.tree.get_children()
        if len(children) > 10:
            self.tree.delete(children[-1])

    def run(self):
        """Start the application (blocking)."""
        # Thread is already started in __init__
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            self.running = False
            self.cleanup()

    def cleanup(self):
        self.running = False
        # Signal the send worker thread to exit gracefully
        try:
            self._send_queue.put_nowait(None)
        except Exception:
            pass
        if hasattr(self, '_send_worker_thread'):
            self._send_worker_thread.join(timeout=2.0)
        # Signal remote decode worker to exit gracefully
        try:
            self._remote_decode_queue.put_nowait(None)
        except Exception:
            pass
        if hasattr(self, '_remote_decode_worker_thread'):
            self._remote_decode_worker_thread.join(timeout=2.0)
        # Signal master compute worker to exit gracefully
        try:
            self._master_compute_queue.put_nowait(None)
        except Exception:
            pass
        if hasattr(self, '_master_compute_worker_thread'):
            self._master_compute_worker_thread.join(timeout=2.0)
        # Flush async DB save queue — wait up to 5 s for in-flight writes to finish
        try:
            self._db_save_queue.put_nowait(None)
        except Exception:
            pass
        if hasattr(self, '_db_save_worker_thread'):
            self._db_save_worker_thread.join(timeout=5.0)
        if self.camera: self.camera.release()
        if self.db: self.db.stop_recording()
        if self.network_server: self.network_server.stop()
        if self.coordinator: self.coordinator.stop()
        cv2.destroyAllWindows()
        if self.root: self.root.quit()

    def update_metrics_gui(self, metrics):
        """Update angle labels with latest metrics."""
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
                label_widget.config(text=f"M:{local_txt} | W:{remote_txt}")
            elif local_val is not None:
                label_widget.config(text=f"{local_val:.2f}{unit}")
            else:
                label_widget.config(text="0.0")


if __name__ == "__main__":
    app = MocapGUI()
    app.run()
