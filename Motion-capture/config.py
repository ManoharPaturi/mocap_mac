# Camera Settings
CAMERA_ID = 0  # Device ID for cv2.VideoCapture (integer: 0, 1, 2, ...)
NETWORK_CAMERA_ID = 'cam_0'  # Network identifier (string: 'cam_0', 'cam_1', ...)
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
FPS = 30

# Detection Settings
MIN_DETECTION_CONFIDENCE = 0.5
MIN_TRACKING_CONFIDENCE = 0.5

# Model Selection
try:
    import torch
    TORCH_AVAILABLE = True
    MPS_ENABLED = bool(getattr(torch.backends, 'mps', None) and torch.backends.mps.is_available())
    CUDA_ENABLED = torch.cuda.is_available()
    if MPS_ENABLED:
        DEVICE = "mps"
    elif CUDA_ENABLED:
        DEVICE = "cuda"
    else:
        DEVICE = "cpu"
except Exception:
    TORCH_AVAILABLE = False
    MPS_ENABLED = False
    CUDA_ENABLED = False
    DEVICE = "cpu"

# Options: 'LITE' (Fastest), 'FULL' (Balanced), 'HEAVY' (Most Accurate)
POSE_MODEL_COMPLEXITY = 'FULL' 

# Multi-Person Settings (set to 1 for single-person use — each additional
# slot multiplies MediaPipe's internal memory allocation for all 3 models)
NUM_POSES = 1
NUM_FACES = 1
NUM_HANDS = 2  # 2 hands per person

# Internal Paths (Do not edit unless moving files)
MODEL_PATHS = {
    'LITE': 'models/pose_landmarker_lite.task',
    'FULL': 'models/pose_landmarker_full.task',
    'HEAVY': 'models/pose_landmarker_heavy.task',
    'FACE': 'models/face_landmarker.task',
    'HAND': 'models/hand_landmarker.task'
}

# Visualization Settings
DRAW_LANDMARKS = True
SHOW_FPS = True
THEME_COLOR = (255, 100, 0)

# Database Settings
DB_TYPE = 'sqlite'  # 'sqlite' or 'postgres'
DB_PATH = 'mocap_data.db'  # For SQLite
POSTGRES_CONNECTION = {  # For PostgreSQL (set DB_TYPE='postgres' to use)
    'host': 'localhost',
    'port': 5432,
    'database': 'mocap',
    'user': 'postgres',
    'password': 'your_password'
}

# ============================================
# VS5 ADVANCED CONFIGURATION
# ============================================

# --- Image Preprocessing ---
# Frame Resizing
MAX_FRAME_WIDTH = 640   # Resize frames wider than this before inference; 640 is enough for MediaPipe and saves ~4x unified-memory per frame

# Gamma Correction
GAMMA_DEFAULT = 1.0     # Default gamma value (1.0 = disabled)
GAMMA_MIN = 1.0         # Minimum gamma value
GAMMA_MAX = 1.4         # Maximum gamma value

# Face-Guided Exposure
FACE_EXPOSURE_TARGET = 140.0    # Target brightness for face region (0-255)
FACE_EXPOSURE_MIN_GAIN = 0.5    # Minimum gain factor (prevents over-darkening)
FACE_EXPOSURE_MAX_GAIN = 2.5    # Maximum gain factor (prevents noise amplification)
FACE_EXPOSURE_MIN_BRIGHTNESS = 10.0  # Minimum face brightness to avoid div-by-zero

# CLAHE (Contrast Limited Adaptive Histogram Equalization)
CLAHE_CLIP_LIMIT = 2.0      # Clip limit (1.0-4.0, higher = more contrast)
CLAHE_TILE_GRID_SIZE = (8, 8)  # Tile grid size for adaptive equalization

# --- Physics Engine ---
# Visibility Thresholds
VISIBILITY_HARD_GATE = 0.5      # Landmarks below this are ignored (latched to previous)
VISIBILITY_MIN_METRIC = 0.5     # Minimum visibility for metric calculations

# 1 Euro Filter Parameters (Optimized for 30 FPS)
FILTER_MIN_CUTOFF = 1.0     # Minimum cutoff frequency (Hz) - smooth slow motion
FILTER_BETA = 0.005         # Speed coefficient - responsiveness during fast motion
FILTER_D_CUTOFF = 1.0       # Derivative cutoff frequency (Hz)

# Physics Constraints
CALIBRATION_FRAMES = 30     # Number of frames to learn skeleton (1 sec @ 30fps)
BONE_LENGTH_STRICTNESS_HIGH_VIS = 0.85  # Constraint strength when visibility > 0.5
BONE_LENGTH_STRICTNESS_LOW_VIS = 0.99   # Constraint strength when visibility < 0.5

# --- Metric Calculations ---
# Outlier Rejection
ANGLE_OUTLIER_BASE_THRESHOLD = 50.0     # Base threshold for angle spikes (degrees)
ANGLE_OUTLIER_VELOCITY_COEFF = 0.1      # Velocity-dependent coefficient

# Velocity Limits
MAX_LINEAR_VELOCITY = 6.0   # Maximum limb velocity (m/s) - sanity check

# Smoothing
SMOOTHING_ALPHA_DEFAULT = 0.5   # Default EMA alpha for metrics
SMOOTHING_ALPHA_FACE = 0.3      # Stronger smoothing for facial metrics

# Normalization
BODY_HEIGHT_MULTIPLIER = 2.0    # Multiply hip-ankle distance by this for full height
IPD_DEFAULT = 0.065             # Default interpupillary distance (meters) if detection fails

# --- Detector Toggles (Default States) ---
ENABLE_FACE_DETECTION = True
ENABLE_HAND_DETECTION = True
ENABLE_FACE_EXPOSURE = False    # Face-guided exposure disabled by default

# --- ROI-Based Pose Cropping ---
# Crop to person bounding box for higher effective resolution
ENABLE_ROI_CROPPING = True          # Enable person-centered ROI cropping
ROI_EXPANSION_FACTOR = 0.25         # Expand bbox by 25% (0.2-0.3 recommended)
ROI_MIN_SIZE = 200                  # Minimum ROI size (pixels)
ROI_TARGET_SIZE = 640               # Resize ROI to this size for inference
ROI_SMOOTHING_ALPHA = 0.7           # Smooth ROI position (0.5-0.8)

# --- Performance ---
# Frame Skip (for low-end hardware)
FRAME_SKIP = 0  # Process every Nth frame (0 = no skip, 1 = every other frame)

# Inference Acceleration
# Backends: 'mps' (macOS Metal via MediaPipe GPU delegate), 'gpu', 'cpu', 'auto'
# NOTE: MPS requires frame height to be a multiple of 16 for CVPixelBuffer on Apple Silicon.
#       detector.py now rounds resized height up to the nearest 16 to satisfy this.
INFERENCE_BACKEND = 'mps'
# Backward-compat toggle used by older code paths (kept for compatibility)
PREFER_GPU_DELEGATE = True

# --- Presets ---
# You can define presets for different scenarios
PRESETS = {
    'indoor': {
        'gamma': 1.2,
        'face_exposure': True,
        'clahe_clip': 2.0
    },
    'outdoor': {
        'gamma': 1.0,
        'face_exposure': False,
        'clahe_clip': 2.0
    },
    'low_light': {
        'gamma': 1.3,
        'face_exposure': True,
        'clahe_clip': 3.0
    },
    'high_speed': {
        'gamma': 1.0,
        'face_exposure': False,
        'clahe_clip': 2.0,
        'enable_face': False,
        'enable_hand': False,
        'filter_beta': 0.01  # More responsive
    }
}

# ============================================
# MULTI-CAMERA SETTINGS
# ============================================

# Multi-Camera Mode
ENABLE_MULTI_CAMERA = False
CAMERA_ROLE = 'single'  # 'single', 'server', or 'master'
NUM_CAMERAS = 2         # Total number of cameras in setup

# ============================================
# COORDINATE SYSTEM CONVENTIONS (LOCKED)
# ============================================
# World Frame W (Right-handed):
#   +X: Right
#   +Y: Up
#   +Z: Forward (away from front camera)
WORLD_FRAME = {
    'handedness': 'right',
    'x_axis': 'right',
    'y_axis': 'up',
    'z_axis': 'forward'
}

# OpenCV Camera Frame C:
#   +X: right in image
#   +Y: down in image
#   +Z: forward from lens
CAMERA_FRAME = {
    'x_axis': 'right',
    'y_axis': 'down',
    'z_axis': 'forward'
}

# Transform camera-centric 3D into world frame convention.
# Current calibration/triangulation outputs OpenCV-style camera frame,
# so Y is flipped to enforce +Y up in world frame.
WORLD_AXIS_TRANSFORM = {
    'flip_x': False,
    'flip_y': True,
    'flip_z': False
}

# Angle conventions for dashboard + analytics.
ANGLE_UNIT = 'degrees'
ANGLE_RANGE = (0.0, 180.0)

# --- GUI Integration Mode (NEW) ---
# Set this based on which laptop you're using:
# 'single' - Normal single-camera mode (default)
# 'server' - Full GUI + broadcasts to network (PC1)
# 'master' - Full GUI + receives from network + dual display (PC2)
MULTI_CAMERA_MODE = 'single'
REMOTE_CAMERA_IP = None  # IP of other laptop (e.g., '10.51.179.228')

# Network Configuration
MASTER_IP = '10.137.227.228'  # IP address of master coordinator
DISCOVERY_PORT = 6000        # Port for camera discovery broadcasts
DATA_PORT = 6001             # Port for frame data transmission
NETWORK_PROTOCOL = 'tcp'     # 'udp' (faster) or 'tcp' (reliable)
NETWORK_JPEG_QUALITY = 45   # Raised for better remote display quality (was 30)
NETWORK_STREAM_WIDTH = 640  # Remote stream width for transmission only
NETWORK_STREAM_HEIGHT = 360  # Remote stream height for transmission only
NETWORK_FRAMERATE_LIMIT = 30 # Cap transmission to this FPS

# Frame Synchronization
# 200ms window tolerates WiFi jitter + Windows/Mac FPS mismatch (25-35fps vs 50-60fps).
# Dynamic narrowing is disabled: at 30fps it would shrink the window to ~16ms, far too
# tight for WiFi links where round-trip latency alone can exceed 20ms.
SYNC_TIME_THRESHOLD_MS = 200.0
SYNC_DYNAMIC_THRESHOLD_ENABLED = False  # Disabled — WiFi jitter makes dynamic narrowing counterproductive
SYNC_DYNAMIC_FACTOR = 0.6               # Fraction of frame interval used for dynamic window (unused while disabled)
SYNC_THRESHOLD_MIN_MS = 20             # Floor for dynamic sync window (ms)
SYNC_THRESHOLD_MAX_MS = 30             # Ceiling for dynamic sync window (ms)
FRAME_BUFFER_SIZE = 30         # ~1s of frames at 30fps — gives sync engine enough history to match
STALE_FRAME_TIMEOUT_MS = 5000  # Drop frames only if older than 5s — tolerates WiFi jitter + clock correction lag

# Calibration
CALIBRATION_FILE = 'calibration.json'
AUTO_LOAD_CALIBRATION = True

# ArUco Calibration
ARUCO_DICT_TYPE = 11  # cv2.aruco.DICT_6X6_250
ARUCO_BOARD_WIDTH = 5
ARUCO_BOARD_HEIGHT = 7
ARUCO_MARKER_LENGTH_M = 0.04
ARUCO_MARKER_SEPARATION_M = 0.01
ARUCO_MIN_IMAGES_INTRINSIC = 15
ARUCO_MIN_IMAGES_STEREO = 10
ARUCO_MIN_MARKERS_PER_IMAGE = 4
REPROJECTION_ERROR_EXCELLENT_PX = 1.0
REPROJECTION_ERROR_ACCEPTABLE_PX = 2.0
REPROJECTION_ERROR_MAX_PX = 3.0

# Data Compression
COMPRESS_NETWORK_DATA = True  # Use msgpack compression

# Multi-View Fusion
TRIANGULATION_MIN_VIEWS = 2      # Minimum cameras to see landmark for triangulation
REPROJECTION_ERROR_THRESHOLD = 15.0  # Max reprojection error (pixels)
CONFIDENCE_WEIGHT_VISIBILITY = 0.6   # Weight for visibility in confidence calculation
CONFIDENCE_WEIGHT_REPROJ = 0.4       # Weight for reprojection error in confidence
OCCLUSION_FILL_ENABLED = True        # Use monocular fallback for occluded landmarks
MONOCULAR_SUBJECT_DISTANCE_M = 2.5  # Approx distance for monocular fallback
STEREO_POINT_MIN_INPUT_CONFIDENCE = 0.5  # Min per-camera landmark confidence used for triangulation
KINEMATICS_MIN_POINT_CONFIDENCE = 0.5    # Min 3D point confidence used in kinematics engine
ENABLE_3D_ONE_EURO_FILTER = True         # Filter 3D joint positions before kinematics

# Level 3 Feedback Loop
FEEDBACK_PORT = 6002             # Port for quality feedback (Master -> Server)
FEEDBACK_ENABLED = True         # Enable quality feedback loop
FEEDBACK_INTERVAL_FRAMES = 10    # Send feedback every N frames

# Clock Synchronization
ENABLE_CLOCK_SYNC = True             # Enable automatic clock offset estimation
CLOCK_SYNC_PORT = 6003               # Port for clock sync ping/pong
CLOCK_SYNC_SAMPLES = 5              # Number of ping samples to collect per sync
CLOCK_SYNC_INTERVAL_SEC = 10         # Re-sync every 30 seconds (0 = sync once at startup)
CLOCK_SYNC_RTT_OUTLIER_FACTOR = 2.0  # Reject samples with RTT > factor * min_rtt

# Message Format
MESSAGE_SCHEMA_VERSION = 2           # Payload schema version for forward compatibility
CALIBRATION_ID = None                # Set after calibration (e.g., 'cal_20260227_1430')

# Latency Instrumentation
ENABLE_LATENCY_TRACKING = True       # Track per-stage pipeline latency
LATENCY_LOG_INTERVAL = 100           # Print latency summary every N frames

# Dataset Pipeline
SAVE_RAW_FRAMES = False              # Save raw JPEG frames to disk during recording
SESSION_EXPORT_FORMAT = 'csv'        # 'csv' or 'json'

# GPU Profiling
ENABLE_GPU_PROFILING = False         # Track GPU inference timing

# ============================================
# ABLATION STUDY — per-fix memory toggles
# ============================================
# Set a flag to True to DISABLE that specific memory fix so you can measure
# its individual RSS contribution (Activity Monitor → Memory column).
#
# Procedure:
#   1. Baseline: all flags False — record RSS after 60s
#   2. Flip ONE flag to True, restart, record RSS again
#   3. Difference = that fix’s contribution
#
# Expected contributions (approximate, 30 fps dual-camera):
#   FRAME_RESIZE:      +200–400 MB  (Metal buffer 4× larger at 1280 vs 640)
#   DISPLAY_THROTTLE:  +30–60 MB   (~1.4 MB combined array × 30 fps vs ×10 fps)
#   PHOTO_REUSE:       +100–200 MB  (2.5 MB Tk alloc per 15 ms poll → GC backlog)
#   CLAHE_DEL:         +20–40 MB   (5-6 intermediate arrays freed 30 ms earlier)
#
ABLATION_DISABLE_FRAME_RESIZE     = False  # True → inference at FRAME_WIDTH (original pre-fix)
ABLATION_DISABLE_DISPLAY_THROTTLE = False  # True → redraw display every frame (not every 3rd)
ABLATION_DISABLE_PHOTO_REUSE      = False  # True → new PhotoImage each Tk poll tick
ABLATION_DISABLE_CLAHE_DEL        = False  # True → skip explicit del of CLAHE intermediates
ABLATION_RSS_LOG_INTERVAL         = 60     # print [ABLATION] RSS line every N frames (0 = off)

