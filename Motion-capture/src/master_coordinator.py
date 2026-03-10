"""
Master Coordinator Module
Runs on the master laptop to discover cameras, receive frame data, and coordinate multi-view fusion.
"""

import zmq
import msgpack
import json
import time
import platform
from typing import Dict, List, Optional, Any, Tuple
from threading import Thread, Event
from collections import defaultdict, deque
from dataclasses import dataclass
import numpy as np
import statistics
from config import (
    DISCOVERY_PORT, DATA_PORT, NUM_CAMERAS, COMPRESS_NETWORK_DATA,
    FRAME_BUFFER_SIZE, SYNC_TIME_THRESHOLD_MS, CALIBRATION_FILE,
    FEEDBACK_PORT, FEEDBACK_ENABLED, FEEDBACK_INTERVAL_FRAMES,
    WORLD_AXIS_TRANSFORM, STEREO_POINT_MIN_INPUT_CONFIDENCE,
    KINEMATICS_MIN_POINT_CONFIDENCE, ENABLE_3D_ONE_EURO_FILTER,
    FILTER_MIN_CUTOFF, FILTER_BETA, FILTER_D_CUTOFF,
    ENABLE_CLOCK_SYNC, CLOCK_SYNC_PORT, CLOCK_SYNC_SAMPLES,
    CLOCK_SYNC_INTERVAL_SEC, CLOCK_SYNC_RTT_OUTLIER_FACTOR,
    SYNC_DYNAMIC_THRESHOLD_ENABLED, SYNC_DYNAMIC_FACTOR, SYNC_THRESHOLD_MIN_MS,
    SYNC_THRESHOLD_MAX_MS, FPS
)
from src.stereo_calibration import StereoCalibration
from src.triangulation import Triangulator
from src.one_euro_filter import OneEuroFilter
from src.kinematics_engine import KinematicsEngine
from src.occlusion_fusion import OcclusionFusionEngine
from src.uncertainty_estimation import UncertaintyEstimator, ErrorMetricsCalculator
from src.advanced_kinematics import AdvancedKinematics
from src.dashboard_monitoring import DashboardMonitor
from src.calculations import Calculations


@dataclass
class CameraInfo:
    """Information about a discovered camera."""
    camera_id: str
    ip: str
    port: int
    last_seen: float


@dataclass
class FrameData:
    """Frame data from a single camera."""
    camera_id: str
    frame_number: int
    timestamp: float  # nanoseconds
    results: Dict[str, Any]
    received_at: float


class MasterCoordinator:
    """
    Master Coordinator for multi-camera setup.
    Discovers cameras, receives frame data, and synchronizes frames across cameras.
    """
    
    def __init__(self, num_cameras: int = NUM_CAMERAS):
        """
        Initialize master coordinator.
        
        Args:
            num_cameras: Expected number of cameras
        """
        self.num_cameras = num_cameras
        self.running = False
        self.discovered_cameras: Dict[str, CameraInfo] = {}
        
        # Frame buffers per camera — pre-register known cameras so get_synchronized_batch()
        # doesn't stall on the len < num_cameras guard before the first remote frame arrives.
        self.frame_buffers: Dict[str, deque] = defaultdict(
            lambda: deque(maxlen=FRAME_BUFFER_SIZE)
        )
        self.frame_buffers['local_cam']  # pre-register Mac camera slot
        self.frame_buffers['cam_0']      # pre-register Windows camera slot
        
        # Threads
        self.discovery_thread = None
        self.data_thread = None
        self.stop_event = Event()
        
        # ZMQ context and sockets
        self.context = zmq.Context()
        self.discovery_socket = None
        self.data_sockets: Dict[str, zmq.Socket] = {}
        
        # Statistics
        self.stats = {
            'frames_received': defaultdict(int),
            'frames_synced': 0,
            'sync_failures': 0,
            'frames_dropped_overflow': defaultdict(int),  # sequence-gap drops per camera
        }

        # Per-camera frame rate tracking (used for adaptive sync threshold)
        self.camera_frame_rates: Dict[str, float] = {}          # camera_id -> measured fps
        self.camera_interval_samples: Dict[str, list] = defaultdict(list)  # recent arrival intervals (s)
        self._last_frame_recv_time: Dict[str, float] = {}       # wall-clock of last arrival
        self.last_buffer_sizes: Dict[str, int] = {}             # snapshot of buffer depths
        
        # Fusion modules (Level 1 & 2)
        self.triangulator: Optional[Triangulator] = None
        self._load_calibration()
        
        # Feedback Loop (Level 3)
        self.feedback_socket = None
        if FEEDBACK_ENABLED:
            # We use the same self.context
            self.feedback_socket = self.context.socket(zmq.PUB)
            self._apply_tcp_low_latency_opts(self.feedback_socket)
            try:
                self.feedback_socket.bind(f"tcp://*:{FEEDBACK_PORT}")
                print(f"[MasterCoordinator] Feedback loop active on port {FEEDBACK_PORT}")
            except Exception as e:
                print(f"[MasterCoordinator] Could not bind feedback port: {e}")
        
        self.frame_count = 0
        self.kinematics_engine = KinematicsEngine()
        self.pose_3d_filters: Dict[int, Dict[str, OneEuroFilter]] = defaultdict(dict)
        self.occlusion_fusion_engine = OcclusionFusionEngine()
        self.advanced_kinematics_engine = AdvancedKinematics()
        self.uncertainty_estimator = UncertaintyEstimator(
            calibration_rms=0.5,
            baseline_m=1.0,
            focal_length_px=1000.0
        )
        self.error_metrics_calculator = ErrorMetricsCalculator(window_size=300)
        self.dashboard_monitor = DashboardMonitor(history_size=300)
        
        # Clock synchronization
        self.clock_offsets: Dict[str, int] = {}  # camera_id -> offset in nanoseconds
        self._clock_sync_history: Dict[str, List[Tuple[float, int, int]]] = defaultdict(list)
        self._clock_rtt_min_ns: Dict[str, int] = {}
        self._clock_sync_thread = None
        self._last_sync_time = 0.0
        
        # Sequence gap detection per camera
        self._last_frame_number: Dict[str, int] = {}
        self._sequence_gaps: Dict[str, int] = defaultdict(int)
        
        # Latency tracking
        self._latency_accum: Dict[str, list] = defaultdict(list)  # stage -> [durations_ms]
        
        # Uncertainty tracking (inter-camera disagreement)
        self._landmark_disagreements: Dict[int, float] = {}  # lm_id -> disagreement_m
        self._gpu_reports: Dict[str, Dict[str, Any]] = {}
        self._latest_camera_jpeg: Dict[str, bytes] = {}
        
        # Occlusion state machine per landmark
        self._occlusion_state: Dict[int, str] = {}  # lm_id -> 'VISIBLE'|'OCCLUDED'|'PREDICTED'
        self._occlusion_last_position: Dict[int, Dict] = {}  # lm_id -> last known 3D pos
        self._occlusion_frames_hidden: Dict[int, int] = defaultdict(int)
        self._occlusion_velocity: Dict[int, np.ndarray] = {}
        self._occlusion_last_timestamp_ns: Dict[int, int] = {}
        self.calibration_rms_px: Optional[float] = None
        self.calibration_intrinsic_mean_error_px: Optional[float] = None
        self.db = None
        self._layer_frame_index = 0
        
        print("[MasterCoordinator] Initialized")

    def compact_results_for_sync(self, results: Dict[str, Any], include_jpeg: bool = False) -> Dict[str, Any]:
        """Keep only fields required for synchronization/triangulation to reduce memory usage."""
        if not isinstance(results, dict):
            return {}

        compact: Dict[str, Any] = {}

        pose = results.get('pose')
        if pose is not None and hasattr(pose, 'pose_landmarks'):
            try:
                if pose.pose_landmarks and len(pose.pose_landmarks) > 0:
                    compact['pose_landmarks'] = [
                        {
                            'x': float(lm.x),
                            'y': float(lm.y),
                            'z': float(lm.z),
                            'visibility': float(getattr(lm, 'visibility', 1.0))
                        }
                        for lm in pose.pose_landmarks[0]
                    ]
            except Exception:
                pass
            try:
                if hasattr(pose, 'pose_world_landmarks') and pose.pose_world_landmarks and len(pose.pose_world_landmarks) > 0:
                    compact['pose_world_landmarks'] = [
                        {
                            'x': float(lm.x),
                            'y': float(lm.y),
                            'z': float(lm.z),
                            'visibility': float(getattr(lm, 'visibility', 0.5))
                        }
                        for lm in pose.pose_world_landmarks[0]
                    ]
            except Exception:
                pass

        # Preserve already-serialized payloads from network paths.
        for key in ('pose_landmarks', 'packet_landmarks', 'pose_world_landmarks', 'world_landmarks'):
            if key in results and results.get(key) is not None and key not in compact:
                compact[key] = results.get(key)

        if 'gpu_compute' in results and isinstance(results.get('gpu_compute'), dict):
            compact['gpu_compute'] = results.get('gpu_compute')

        if include_jpeg and results.get('frame_jpeg'):
            compact['frame_jpeg'] = results.get('frame_jpeg')

        return compact

    def get_latest_camera_jpeg(self, camera_id: str) -> Optional[bytes]:
        """Return latest JPEG for a camera without retaining it in sync frame buffers."""
        return self._latest_camera_jpeg.get(camera_id)

    def get_memory_status(self) -> Dict[str, Any]:
        """Best-effort process + queue/buffer memory diagnostics."""
        rss_mb = None
        try:
            if platform.system() != 'Windows':
                import resource
                ru = resource.getrusage(resource.RUSAGE_SELF)
                rss_kb = float(ru.ru_maxrss)
                if platform.system() == 'Darwin':
                    rss_mb = rss_kb / (1024.0 * 1024.0)
                else:
                    rss_mb = rss_kb / 1024.0
        except Exception:
            rss_mb = None

        return {
            'rss_mb': float(rss_mb) if rss_mb is not None else None,
            'frame_buffer_sizes': {cam_id: len(buf) for cam_id, buf in self.frame_buffers.items()},
            'clock_history_sizes': {cam_id: len(hist) for cam_id, hist in self._clock_sync_history.items()},
            'latest_jpeg_cameras': list(self._latest_camera_jpeg.keys()),
            'camera_frame_rates': dict(self.camera_frame_rates),
            'frames_dropped_overflow': dict(self.stats['frames_dropped_overflow']),
            'last_buffer_sizes': dict(self.last_buffer_sizes),
        }

    def attach_database(self, db):
        """Attach MocapDB instance for layered persistence (optional)."""
        self.db = db

    def reset_layer_frame_index(self):
        """Reset Layer-0 frame index at recording session start."""
        self._layer_frame_index = 0

    def save_frame_to_database(self, synced_frames: List['FrameData'], pose_3d: Dict[str, Any]):
        """Persist one synchronized batch into Layer 0-3 tables."""
        if not self.db or not getattr(self.db, 'running', False):
            return
        if not synced_frames:
            return
        try:
            # Runtime timestamps are nanoseconds in FrameData
            timestamp_ms = float(synced_frames[0].timestamp) / 1_000_000.0
            frame_id = self.db.save_frame_metadata(timestamp_ms, self._layer_frame_index, len(synced_frames))
            if frame_id is None:
                return

            # Layer 1: raw 2D landmarks per camera
            for frame in synced_frames:
                raw_lms = self._extract_pose_landmarks(frame.results) or []
                self.db.save_raw_landmarks(frame_id, frame.camera_id, raw_lms, timestamp_ms)

            # Layer 2: 3D joints
            joints_payload = {}
            pose_points = pose_3d.get('pose_3d', {}) if isinstance(pose_3d, dict) else {}
            for lm_id, point in (pose_points or {}).items():
                joints_payload[int(lm_id)] = {
                    'x': point.get('x', 0.0),
                    'y': point.get('y', 0.0),
                    'z': point.get('z', 0.0),
                    'confidence': point.get('visibility', 0.0),
                    'reprojection_error_px': point.get('reproj_error')
                }
            self.db.save_joints_3d(frame_id, joints_payload, timestamp_ms, '1.0', 'opencv_triangulate')

            # Layer 3: kinematics angles
            angle_map = {}
            if isinstance(pose_3d, dict):
                kinematics = pose_3d.get('kinematics_3d', {})
                if isinstance(kinematics, dict):
                    angle_map = kinematics.get('joint_angles_deg', {}) or {}

            # Fallback: compute angles directly from 3D joints if kinematics missing
            if not angle_map and joints_payload:
                max_joint_id = max(joints_payload.keys()) if joints_payload else -1
                pose_list = [None] * max(max_joint_id + 1, 33)
                for lm_id, joint in joints_payload.items():
                    if lm_id < len(pose_list):
                        pose_list[lm_id] = {
                            'x': joint.get('x', 0.0),
                            'y': joint.get('y', 0.0),
                            'z': joint.get('z', 0.0),
                            'v': joint.get('confidence', 0.0)
                        }
                computed_metrics = Calculations.get_body_metrics(pose_list)
                angle_map = {
                    k: v for k, v in (computed_metrics or {}).items()
                    if isinstance(v, (int, float)) and k.startswith('Angle_')
                }

            self.db.save_kinematics_3d(frame_id, angle_map, timestamp_ms, '1.0', '1.0')

            self._layer_frame_index += 1
        except Exception as e:
            print(f"[MasterCoordinator] Layered DB persistence skipped: {e}")

    def _load_calibration(self):
        """Load stereo calibration file and initialize triangulator."""
        try:
            calibration = StereoCalibration()
            try:
                calibration.load_calibration(CALIBRATION_FILE)
                print(f"[MasterCoordinator] Loaded calibration from {CALIBRATION_FILE}")

                # Calibration quality metrics for dashboard
                rms = None
                if hasattr(calibration, 'metadata') and isinstance(calibration.metadata, dict):
                    rms_val = calibration.metadata.get('rms_error')
                    if rms_val is not None:
                        try:
                            rms = float(rms_val)
                        except Exception:
                            rms = None
                self.calibration_rms_px = rms

                intrinsic_errors = [
                    float(cam.reprojection_error)
                    for cam in calibration.cameras.values()
                    if cam.reprojection_error is not None and float(cam.reprojection_error) > 0
                ]
                self.calibration_intrinsic_mean_error_px = (
                    float(np.mean(intrinsic_errors)) if intrinsic_errors else None
                )
            except (FileNotFoundError, Exception):
                print("[MasterCoordinator] No calibration file found. Using DEFAULT calibration (1.0m baseline).")
                calibration.create_default_calibration(width=1280, height=720)
                self.calibration_rms_px = None
                self.calibration_intrinsic_mean_error_px = None
            self.triangulator = Triangulator(calibration)
        except Exception as e:
            print(f"[MasterCoordinator] Failed to initialize triangulator: {e}")
            self.triangulator = None

    def reload_calibration(self, filepath: Optional[str] = None):
        """Reload calibration from disk (e.g. after ArUco wizard saves a new file).

        Args:
            filepath: Override path. Defaults to CALIBRATION_FILE from config.
        """
        import importlib, config as _cfg
        importlib.reload(_cfg)  # pick up any runtime config changes
        target = filepath or CALIBRATION_FILE
        print(f"[MasterCoordinator] Reloading calibration from {target}")
        try:
            calibration = StereoCalibration()
            calibration.load_calibration(target)
            rms = None
            if hasattr(calibration, 'metadata') and isinstance(calibration.metadata, dict):
                try:
                    rms = float(calibration.metadata.get('rms_error') or 0) or None
                except Exception:
                    rms = None
            self.calibration_rms_px = rms
            intrinsic_errors = [
                float(cam.reprojection_error)
                for cam in calibration.cameras.values()
                if cam.reprojection_error is not None and float(cam.reprojection_error) > 0
            ]
            self.calibration_intrinsic_mean_error_px = (
                float(np.mean(intrinsic_errors)) if intrinsic_errors else None
            )
            self.triangulator = Triangulator(calibration)
            cam_ids = list(calibration.cameras.keys())
            print(f"[MasterCoordinator] Calibration reloaded. Cameras: {cam_ids}  RMS: {rms}")
            return True
        except Exception as e:
            print(f"[MasterCoordinator] Failed to reload calibration: {e}")
            return False

    def get_calibration_quality_metrics(self) -> Dict[str, Any]:
        """Return calibration quality indicators for UI/dashboard."""
        rms = self.calibration_rms_px
        intrinsic_mean = self.calibration_intrinsic_mean_error_px
        effective = rms if rms is not None else intrinsic_mean
        return {
            'calibration_rms_px': rms,
            'calibration_intrinsic_mean_error_px': intrinsic_mean,
            'calibration_effective_error_px': effective,
            'calibration_pass_lt_1px': bool(effective is not None and effective < 1.0),
        }

    def start(self):
        """Start the master coordinator."""
        if self.running:
            print("[MasterCoordinator] Already running")
            return
        
        self.running = True
        self.stop_event.clear()
        
        # Setup discovery socket
        self._setup_discovery_socket()
        
        # Start discovery listener thread
        self.discovery_thread = Thread(target=self._discovery_listener, daemon=True)
        self.discovery_thread.start()
        
        # Start data receiver thread
        self.data_thread = Thread(target=self._data_receiver, daemon=True)
        self.data_thread.start()

        if ENABLE_CLOCK_SYNC and CLOCK_SYNC_INTERVAL_SEC > 0:
            self._clock_sync_thread = Thread(target=self._clock_sync_loop, daemon=True)
            self._clock_sync_thread.start()
        
        print(f"[MasterCoordinator] Started, listening for {self.num_cameras} cameras")
    
    def stop(self):
        """Stop the master coordinator."""
        if not self.running:
            return
        
        print("[MasterCoordinator] Stopping...")
        self.running = False
        self.stop_event.set()
        
        # Wait for threads
        if self.discovery_thread:
            self.discovery_thread.join(timeout=2.0)
        if self.data_thread:
            self.data_thread.join(timeout=2.0)
        if self._clock_sync_thread:
            self._clock_sync_thread.join(timeout=2.0)
        
        # Close sockets
        if self.discovery_socket:
            self.discovery_socket.close()
        
        for socket in self.data_sockets.values():
            socket.close()
        
        self.context.term()
        print("[MasterCoordinator] Stopped")

    def _clock_sync_loop(self):
        """Periodic clock re-synchronization loop."""
        while self.running and not self.stop_event.is_set():
            try:
                self.check_resync_needed()
            except Exception as e:
                if self.running:
                    print(f"[ClockSync] Periodic sync loop error: {e}")
            time.sleep(1.0)

    def _get_sync_threshold_ns(self) -> int:
        """Compute effective sync threshold in nanoseconds."""
        base_ms = float(SYNC_TIME_THRESHOLD_MS)
        if not SYNC_DYNAMIC_THRESHOLD_ENABLED:
            return int(base_ms * 1_000_000)

        # Use measured per-camera rates if available, else fall back to config FPS
        effective_ms = self.get_effective_sync_threshold_ms()
        return int(effective_ms * 1_000_000)

    def get_current_sync_threshold_ms(self) -> float:
        """Expose current effective sync threshold in milliseconds."""
        return self._get_sync_threshold_ns() / 1_000_000.0

    def get_effective_sync_threshold_ms(self) -> float:
        """Compute sync threshold from the slowest camera's measured frame interval.

        Falls back to the static SYNC_TIME_THRESHOLD_MS when no rate data is available
        (startup) or when SYNC_DYNAMIC_THRESHOLD_ENABLED is False.
        When dynamic mode is on, the result is clamped to [SYNC_THRESHOLD_MIN_MS,
        SYNC_THRESHOLD_MAX_MS] so a janky WiFi link never shrinks the window below
        what a single frame interval looks like on the slowest camera.
        """
        if not SYNC_DYNAMIC_THRESHOLD_ENABLED or not self.camera_frame_rates:
            return float(SYNC_TIME_THRESHOLD_MS)
        min_fps = min((fps for fps in self.camera_frame_rates.values() if fps > 0), default=None)
        if not min_fps:
            return float(SYNC_TIME_THRESHOLD_MS)
        interval_ms = 1000.0 / min_fps
        dynamic_ms = interval_ms * float(SYNC_DYNAMIC_FACTOR)
        return max(float(SYNC_THRESHOLD_MIN_MS), min(float(SYNC_THRESHOLD_MAX_MS), dynamic_ms))
    
    def _setup_discovery_socket(self):
        """Setup ZMQ socket for receiving discovery broadcasts."""
        # NOTE: Discovery uses TCP instead of UDP because ZMQ doesn't support
        # UDP with PUB/SUB sockets. We'll need to know camera IPs in advance
        # or use a different discovery mechanism.
        # For now, we'll connect to known camera IPs
        pass  # Discovery will be handled differently

    def _apply_tcp_low_latency_opts(self, sock: zmq.Socket):
        """
        Apply TCP options to reduce packet coalescing stalls on WiFi links.
        Safe-noop on platforms/libzmq builds that don't expose these options.
        """
        try:
            if hasattr(zmq, 'TCP_NODELAY'):
                sock.setsockopt(zmq.TCP_NODELAY, 1)
            else:
                # Fallback used in some builds where constant is not exposed.
                # 96 corresponds to TCP_NODELAY in libzmq socket options.
                sock.setsockopt(96, 1)
            if hasattr(zmq, 'TCP_KEEPALIVE'):
                sock.setsockopt(zmq.TCP_KEEPALIVE, 1)
            if hasattr(zmq, 'TCP_KEEPALIVE_IDLE'):
                sock.setsockopt(zmq.TCP_KEEPALIVE_IDLE, 10)
        except Exception:
            # Keep compatibility if socket option isn't supported by runtime.
            pass
    
    def _discovery_listener(self):
        """
        Listen for camera connections.
        Since ZMQ PUB/SUB doesn't support UDP broadcast, we use a simpler approach:
        - Cameras are configured with known IPs in config.py
        - Master connects to these IPs directly
        """
        # This method is kept for compatibility but discovery is now manual
        # via discover_cameras_manual()
        while self.running and not self.stop_event.is_set():
            time.sleep(1.0)
    
    def discover_cameras_manual(self, camera_ips: List[str]):
        """
        Manually connect to cameras at specified IP addresses.
        
        Args:
            camera_ips: List of camera IP addresses (e.g., ['192.168.1.101', '192.168.1.102'])
        """
        for i, ip in enumerate(camera_ips):
            camera_id = f"cam_{i}"
            print(f"[MasterCoordinator] Connecting to camera at {ip}...")
            
            # Try to connect to discovery port first to verify camera is alive
            try:
                test_socket = self.context.socket(zmq.SUB)
                self._apply_tcp_low_latency_opts(test_socket)
                test_socket.setsockopt(zmq.RCVTIMEO, 2000)  # 2 second timeout
                test_socket.connect(f"tcp://{ip}:{DISCOVERY_PORT}")
                test_socket.setsockopt(zmq.SUBSCRIBE, b"")
                
                # Try to receive one message
                try:
                    data = test_socket.recv()
                    if COMPRESS_NETWORK_DATA:
                        msg = msgpack.unpackb(data)
                    else:
                        msg = json.loads(data.decode('utf-8'))
                    
                    if msg.get('type') == 'discovery':
                        camera_id = msg.get('camera_id', camera_id)
                        print(f"[MasterCoordinator] Discovered camera: {camera_id} at {ip}")
                except zmq.Again:
                    # Timeout, but camera might still be there
                    print(f"[MasterCoordinator] No discovery message, but connecting anyway...")
                
                test_socket.close()
                
                # Add to discovered cameras
                self.discovered_cameras[camera_id] = CameraInfo(
                    camera_id=camera_id,
                    ip=ip,
                    port=DATA_PORT,
                    last_seen=time.time()
                )
                
                # Connect to data stream
                self._connect_to_camera(camera_id, ip, DATA_PORT)
                
            except Exception as e:
                print(f"[MasterCoordinator] Failed to connect to {ip}: {e}")
        
        # Run clock synchronization after all cameras are connected
        if ENABLE_CLOCK_SYNC:
            try:
                self.sync_camera_clocks(camera_ips, 
                                         [info.camera_id for info in self.discovered_cameras.values()])
            except Exception as e:
                print(f"[ClockSync] ⚠️  Clock synchronization failed: {e}")
                print(f"[ClockSync] Continuing without clock correction.")
    
    def estimate_clock_offset(self, camera_ip: str, camera_id: str) -> Optional[int]:
        """
        Estimate clock offset to a remote camera using Cristian's Algorithm.
        
        Sends CLOCK_SYNC_SAMPLES ping requests and uses the median of non-outlier
        round-trip samples to compute the offset.
        
        Args:
            camera_ip: IP address of the camera server
            camera_id: Camera identifier for logging
            
        Returns:
            Clock offset in nanoseconds (add to camera timestamps to align with master),
            or None if sync failed.
        """
        samples = []  # list of (rtt_ns, offset_ns)
        
        for i in range(CLOCK_SYNC_SAMPLES):
            # Create a fresh REQ socket for each sample to avoid
            # state corruption after recv timeouts (REQ enforces
            # strict send→recv→send→recv ordering).
            sock = self.context.socket(zmq.REQ)
            self._apply_tcp_low_latency_opts(sock)
            sock.setsockopt(zmq.LINGER, 0)
            sock.setsockopt(zmq.RCVTIMEO, 500)   # 500ms timeout per sample — failure path costs 500ms×N instead of 2000ms×N
            sock.connect(f"tcp://{camera_ip}:{CLOCK_SYNC_PORT}")
            
            try:
                ping_msg = {
                    'type': 'clock_ping',
                    'master_time_ns': time.time_ns(),
                    'sample': i
                }
                t0 = time.time_ns()
                sock.send(msgpack.packb(ping_msg))
                
                try:
                    reply_data = sock.recv()
                    t1 = time.time_ns()
                    reply = msgpack.unpackb(reply_data, raw=False)
                    
                    if reply.get('type') == 'clock_pong':
                        server_time = reply['server_time_ns']
                        rtt = t1 - t0
                        # Cristian's Algorithm: offset = server_time - (t0 + RTT/2)
                        offset = server_time - (t0 + rtt // 2)
                        samples.append((rtt, offset))
                except zmq.Again:
                    print(f"[ClockSync] Timeout on sample {i} for {camera_id}")
                    continue
                
                time.sleep(0.01)  # Small delay between samples
            finally:
                sock.close()
        
        if not samples:
            print(f"[ClockSync] No valid samples for {camera_id}")
            return None
        
        # Reject RTT outliers: keep only samples with RTT <= factor * min_rtt
        min_rtt = min(s[0] for s in samples)
        threshold = min_rtt * CLOCK_SYNC_RTT_OUTLIER_FACTOR
        good_samples = [s for s in samples if s[0] <= threshold]
        
        if not good_samples:
            good_samples = samples  # Fallback: use all if filter is too strict
        
        # Use median offset for robustness
        offsets = [s[1] for s in good_samples]
        median_offset = int(statistics.median(offsets))
        
        offset_ms = median_offset / 1_000_000
        min_rtt_ms = min_rtt / 1_000_000
        self._clock_rtt_min_ns[camera_id] = int(min_rtt)
        self._clock_sync_history[camera_id].append((time.time(), int(median_offset), int(min_rtt)))
        if len(self._clock_sync_history[camera_id]) > 120:
            self._clock_sync_history[camera_id] = self._clock_sync_history[camera_id][-60:]
        print(f"[ClockSync] {camera_id}: offset = {offset_ms:+.2f} ms "
              f"(RTT min = {min_rtt_ms:.2f} ms, {len(good_samples)}/{len(samples)} samples used)")
        
        return median_offset
    
    def sync_camera_clocks(self, camera_ips: List[str], camera_ids: List[str]):
        """
        Estimate clock offsets for all connected cameras.
        
        Args:
            camera_ips: List of camera IP addresses
            camera_ids: List of corresponding camera IDs
        """
        print(f"[ClockSync] Starting clock synchronization for {len(camera_ips)} camera(s)...")
        
        for ip, cam_id in zip(camera_ips, camera_ids):
            offset = self.estimate_clock_offset(ip, cam_id)
            if offset is not None:
                self.clock_offsets[cam_id] = offset
                print(f"[ClockSync] ✅ {cam_id} synchronized (offset: {offset / 1_000_000:+.2f} ms)")
            else:
                print(f"[ClockSync] ⚠️  {cam_id} sync failed — timestamps will be uncorrected")
        
        self._last_sync_time = time.time()
        
        if self.clock_offsets:
            print(f"[ClockSync] Synchronization complete. {len(self.clock_offsets)} camera(s) corrected.")
        else:
            print(f"[ClockSync] No cameras synchronized. Falling back to raw timestamps.")
    
    def _check_sync_quality(self, synced_frames: List) -> Dict[str, float]:
        """
        Diagnostic: analyze timestamp spread in a synchronized batch.
        
        Args:
            synced_frames: List of FrameData from get_synchronized_batch()
            
        Returns:
            Dict with 'spread_ms' and per-camera offsets from mean
        """
        if not synced_frames or len(synced_frames) < 2:
            return {}
        
        timestamps = [f.timestamp for f in synced_frames]
        mean_ts = sum(timestamps) / len(timestamps)
        spread_ns = max(timestamps) - min(timestamps)
        spread_ms = spread_ns / 1_000_000
        
        per_camera = {}
        for f in synced_frames:
            offset_ms = (f.timestamp - mean_ts) / 1_000_000
            per_camera[f.camera_id] = offset_ms
        
        if spread_ms > 10.0:
            print(f"[ClockSync] ⚠️  Timestamp spread = {spread_ms:.1f} ms (>10 ms) — "
                  f"consider re-synchronizing clocks")
            for cam_id, off_ms in per_camera.items():
                print(f"  {cam_id}: {off_ms:+.2f} ms from mean")
        
        return {'spread_ms': spread_ms, 'per_camera_offset_ms': per_camera}
    
    def check_resync_needed(self) -> bool:
        """
        Check if periodic clock re-synchronization is needed.
        
        Should be called periodically (e.g., in the main processing loop).
        If CLOCK_SYNC_INTERVAL_SEC has elapsed since the last sync, triggers
        a re-sync for all connected cameras.
        
        Returns:
            True if re-sync was performed, False otherwise.
        """
        if not ENABLE_CLOCK_SYNC or CLOCK_SYNC_INTERVAL_SEC <= 0:
            return False
        
        elapsed = time.time() - self._last_sync_time
        if elapsed < CLOCK_SYNC_INTERVAL_SEC:
            return False
        
        print(f"[ClockSync] Periodic re-sync triggered ({elapsed:.0f}s since last sync)")
        camera_ips = [info.ip for info in self.discovered_cameras.values()]
        camera_ids = [info.camera_id for info in self.discovered_cameras.values()]
        
        if camera_ips:
            self.sync_camera_clocks(camera_ips, camera_ids)
            return True
        return False

    def get_clock_sync_status(self) -> Dict[str, Dict[str, float]]:
        """Return per-camera clock sync diagnostics for dashboard display."""
        status: Dict[str, Dict[str, float]] = {}
        now = time.time()

        for cam_id, offset_ns in self.clock_offsets.items():
            hist = self._clock_sync_history.get(cam_id, [])
            drift_ms_per_min = 0.0
            last_sync_age_s = -1.0
            samples = len(hist)

            if hist:
                last_sync_age_s = max(0.0, now - hist[-1][0])
            if len(hist) >= 2:
                t0, off0, _ = hist[0]
                t1, off1, _ = hist[-1]
                dt = t1 - t0
                if dt > 1e-6:
                    drift_ns_per_sec = (off1 - off0) / dt
                    drift_ms_per_min = (drift_ns_per_sec * 60.0) / 1_000_000.0

            status[cam_id] = {
                'offset_ms': float(offset_ns) / 1_000_000.0,
                'rtt_min_ms': float(self._clock_rtt_min_ns.get(cam_id, 0)) / 1_000_000.0,
                'drift_ms_per_min': float(drift_ms_per_min),
                'last_sync_age_s': float(last_sync_age_s),
                'samples': float(samples),
            }

        return status
    
    def _process_discovery(self, msg: Dict[str, Any]):
        """Process a camera discovery message."""
        camera_id = msg.get('camera_id')
        ip = msg.get('ip')
        port = msg.get('port')
        
        if not camera_id or not ip or not port:
            return
        
        # Update or add camera info
        if camera_id not in self.discovered_cameras:
            print(f"[MasterCoordinator] Discovered camera: {camera_id} at {ip}:{port}")
            
            # Connect to camera's data stream
            self._connect_to_camera(camera_id, ip, port)
        
        # Update last seen time
        self.discovered_cameras[camera_id] = CameraInfo(
            camera_id=camera_id,
            ip=ip,
            port=port,
            last_seen=time.time()
        )
    
    def _connect_to_camera(self, camera_id: str, ip: str, port: int):
        """Connect to a camera's data stream."""
        try:
            socket = self.context.socket(zmq.SUB)
            self._apply_tcp_low_latency_opts(socket)
            socket.setsockopt(zmq.RCVTIMEO, 1000)  # 1s receive timeout
            socket.setsockopt(zmq.LINGER, 0)        # Don't block on close
            socket.setsockopt(zmq.RCVHWM, 5)        # Small buffer so frames survive brief Mac lag
            socket.setsockopt(zmq.CONFLATE, 0)       # Preserve all buffered frames for sync matching
            socket.connect(f"tcp://{ip}:{port}")
            socket.setsockopt(zmq.SUBSCRIBE, b"")  # Subscribe to all topics
            # ZMQ slow joiner fix: give the SUB socket time to stabilize
            time.sleep(0.5)
            self.data_sockets[camera_id] = socket
            print(f"[MasterCoordinator] Connected to {camera_id} data stream at {ip}:{port}")
        except Exception as e:
            print(f"[MasterCoordinator] Error connecting to {camera_id}: {e}")
    
    def _data_receiver(self):
        """Receive frame data from all connected cameras."""
        print("[MasterCoordinator] Data receiver started")
        msg_count = 0
        last_log_time = time.time()
        poller = zmq.Poller()
        socket_to_camera_id = {}
        last_socket_count = -1
        
        while self.running and not self.stop_event.is_set():
            try:
                if not self.data_sockets:
                    time.sleep(0.1)  # Wait until sockets are connected
                    continue

                if len(self.data_sockets) != last_socket_count:
                    poller = zmq.Poller()
                    socket_to_camera_id = {}
                    for camera_id, socket in list(self.data_sockets.items()):
                        poller.register(socket, zmq.POLLIN)
                        socket_to_camera_id[socket] = camera_id
                    last_socket_count = len(self.data_sockets)

                ready = dict(poller.poll(timeout=10))

                for socket, event in ready.items():
                    if not (event & zmq.POLLIN):
                        continue

                    camera_id = socket_to_camera_id.get(socket)
                    if camera_id is None:
                        continue

                    # Process buffered frames — cap at 3 per poll cycle to avoid
                    # CPU starvation of the video capture thread.
                    for _ in range(3):
                        try:
                            raw_data = socket.recv(zmq.NOBLOCK)
                        except zmq.Again:
                            break

                        msg_count += 1
                        if msg_count <= 5:
                            print(f"[MasterCoordinator] ✅ Receiving data from {camera_id} (msg #{msg_count})")

                        if COMPRESS_NETWORK_DATA:
                            msg = msgpack.unpackb(raw_data, raw=False)
                        else:
                            msg = json.loads(raw_data.decode('utf-8'))

                        if msg.get('type') == 'frame_data':
                            self._process_frame_data(msg)

                # Periodic heartbeat log every 5 seconds
                now = time.time()
                if now - last_log_time >= 5.0:
                    total = sum(self.stats['frames_received'].values())
                    print(f"[MasterCoordinator] Heartbeat: {total} total frames received from {list(self.data_sockets.keys())}")
                    last_log_time = now
                
            except Exception as e:
                if self.running:
                    print(f"[MasterCoordinator] Data receiver error: {e}")

    # =========================================================================
    # Latency Instrumentation
    # =========================================================================
    
    def record_latency(self, stage: str, duration_ms: float):
        """
        Record a latency sample for a named pipeline stage.
        
        Stages: 'capture', 'detection', 'network', 'sync', 'triangulation',
                'filtering', 'kinematics', 'display', 'total'
        """
        self._latency_accum[stage].append(duration_ms)
        # Keep bounded
        if len(self._latency_accum[stage]) > 500:
            self._latency_accum[stage] = self._latency_accum[stage][-250:]

        # Mirror into dashboard monitoring module
        try:
            if stage == 'capture':
                self.dashboard_monitor.latency.begin_frame()
            self.dashboard_monitor.latency.record_stage(stage, duration_ms)
            if stage == 'total':
                self.dashboard_monitor.latency.end_frame()
        except Exception:
            pass
    
    def get_latency_stats(self) -> Dict[str, Dict[str, float]]:
        """
        Get latency statistics per pipeline stage.
        
        Returns dict: stage -> {'mean_ms': ..., 'p95_ms': ..., 'max_ms': ...}
        """
        stats = {}
        for stage, samples in self._latency_accum.items():
            if not samples:
                continue
            arr = sorted(samples)
            n = len(arr)
            stats[stage] = {
                'mean_ms': sum(arr) / n,
                'p95_ms': arr[int(n * 0.95)] if n >= 20 else arr[-1],
                'max_ms': arr[-1],
                'samples': n
            }
        return stats
    
    def _log_latency_summary(self):
        """Print latency summary to console (called periodically)."""
        from config import ENABLE_LATENCY_TRACKING
        if not ENABLE_LATENCY_TRACKING:
            return
        stats = self.get_latency_stats()
        if not stats:
            return
        lines = ["[Latency Summary]"]
        for stage, s in sorted(stats.items()):
            lines.append(f"  {stage:15s}: mean={s['mean_ms']:.1f}ms  "
                         f"p95={s['p95_ms']:.1f}ms  max={s['max_ms']:.1f}ms  (n={s['samples']})")
        print("\n".join(lines))

    def _process_frame_data(self, msg: Dict[str, Any]):
        """Process incoming frame data from a camera."""
        t_recv = time.time_ns()
        camera_id = msg.get('camera_id')
        frame_number = msg.get('frame_number')
        timestamp = msg.get('timestamp')
        results = msg.get('results')
        packet_landmarks = msg.get('landmarks')
        
        # Schema version check
        schema_ver = msg.get('schema_version', 1)
        if schema_ver > 2:
            print(f"[MasterCoordinator] Warning: unknown schema v{schema_ver} from {camera_id}")
        
        # Allow empty results dict (no detection) - still valid for sync.
        # Use explicit None checks instead of truthiness so timestamp=0 or
        # empty-string camera_id don't silently drop valid frames.
        if camera_id is None or frame_number is None or timestamp is None or results is None:
            if self.stats['frames_received'].get(camera_id, 0) < 3:
                print(f"[MasterCoordinator] Skipping invalid frame from {camera_id}")
            return

        # Network latency monitoring (after receiving, before clock correction)
        recv_time_ms = time.time() * 1000.0
        raw_latency_ms = recv_time_ms - timestamp / 1_000_000.0  # timestamp is ns → convert to ms
        if ENABLE_CLOCK_SYNC and camera_id in self.clock_offsets:
            corrected_latency_ms = raw_latency_ms + self.clock_offsets[camera_id] / 1_000_000.0
        else:
            corrected_latency_ms = raw_latency_ms
        if corrected_latency_ms > 300:
            frames_seen = self.stats['frames_received'].get(camera_id, 0)
            if frames_seen % 30 == 0:  # throttle: log once per ~1s at 30fps
                print(f"[Latency] ⚠️  {camera_id}: {corrected_latency_ms:.0f}ms network delay "
                      f"(raw={raw_latency_ms:.0f}ms)")

        # Sequence gap detection
        if camera_id in self._last_frame_number:
            expected = self._last_frame_number[camera_id] + 1
            if frame_number > expected:
                gap = frame_number - expected
                self._sequence_gaps[camera_id] += gap
                self.stats['frames_dropped_overflow'][camera_id] += gap
                if self._sequence_gaps[camera_id] <= 5:  # Only warn first few
                    print(f"[Sync] {camera_id}: sequence gap detected "
                          f"(expected #{expected}, got #{frame_number}, {gap} frames dropped)")
        self._last_frame_number[camera_id] = frame_number

        # Per-camera frame rate measurement (arrival-time intervals)
        recv_wall = time.time()
        if camera_id in self._last_frame_recv_time:
            interval_s = recv_wall - self._last_frame_recv_time[camera_id]
            if 0 < interval_s < 1.0:  # ignore gaps > 1s (startup, reconnect)
                samples = self.camera_interval_samples[camera_id]
                samples.append(interval_s)
                if len(samples) > 30:
                    del samples[:-30]
                if len(samples) >= 5:
                    avg = sum(samples) / len(samples)
                    self.camera_frame_rates[camera_id] = 1.0 / avg if avg > 0 else 0.0
        self._last_frame_recv_time[camera_id] = recv_wall
        self.last_buffer_sizes[camera_id] = len(self.frame_buffers.get(camera_id, []))

        if packet_landmarks is not None:
            results['packet_landmarks'] = packet_landmarks

        gpu_compute = msg.get('gpu_compute')
        if isinstance(gpu_compute, dict):
            results['gpu_compute'] = gpu_compute
            self._gpu_reports[camera_id] = gpu_compute
        
        # Keep JPEG out of sync buffers (stored separately as latest-only)
        if 'frame_jpeg' in msg and msg['frame_jpeg']:
            self._latest_camera_jpeg[camera_id] = msg['frame_jpeg']
        
        # Create FrameData object
        frame_data = FrameData(
            camera_id=camera_id,
            frame_number=frame_number,
            timestamp=timestamp,
            results=self.compact_results_for_sync(results, include_jpeg=False),
            received_at=time.time()
        )
        
        # Apply clock offset correction if available
        if ENABLE_CLOCK_SYNC and camera_id in self.clock_offsets:
            frame_data.timestamp = timestamp + self.clock_offsets[camera_id]
        
        # Add to buffer
        self.frame_buffers[camera_id].append(frame_data)
        self.stats['frames_received'][camera_id] += 1
        
        # Debug only first 3 frames
        if self.stats['frames_received'][camera_id] <= 3:
            has_jpeg = bool(msg.get('frame_jpeg'))  # jpeg lives in msg, not in results
            print(f"[MasterCoordinator] Buffered frame {frame_number} from {camera_id} (JPEG: {has_jpeg})")
    
    def get_synchronized_batch(self) -> Optional[List[FrameData]]:
        """
        Get a synchronized batch of frames from all cameras.
        Returns None if not all cameras have matching frames.
        
        Includes stale-frame eviction: if a camera's newest frame is older
        than STALE_FRAME_TIMEOUT_MS compared to the freshest camera, its
        buffer is cleared to prevent blocking sync forever when a camera
        drops out.
        
        Returns:
            List of FrameData objects, one per camera, with matching timestamps
        """
        from config import STALE_FRAME_TIMEOUT_MS
        
        if len(self.frame_buffers) < self.num_cameras:
            # Not all cameras connected yet
            return None
        
        # --- Stale frame eviction ---
        # Use received_at (Mac wall-clock seconds) so inter-machine clock skew
        # (Windows vs Mac) never causes a false-stale eviction.
        global_newest_recv = None
        for camera_id, buffer in self.frame_buffers.items():
            if len(buffer) > 0:
                if global_newest_recv is None or buffer[-1].received_at > global_newest_recv:
                    global_newest_recv = buffer[-1].received_at

        if global_newest_recv is not None:
            stale_threshold_s = STALE_FRAME_TIMEOUT_MS / 1000.0
            for camera_id, buffer in list(self.frame_buffers.items()):
                if camera_id == 'local_cam':
                    continue
                if len(buffer) > 0:
                    age_s = global_newest_recv - buffer[-1].received_at
                    if age_s > stale_threshold_s:
                        stale_ms = age_s * 1000.0
                        if not hasattr(self, '_stale_warned'):
                            self._stale_warned = set()
                        if camera_id not in self._stale_warned:
                            print(f"[Sync] ⚠️  {camera_id} stale by {stale_ms:.0f}ms — "
                                  f"clearing {len(buffer)} buffered frames")
                            self._stale_warned.add(camera_id)
                        buffer.clear()
                    else:
                        # Camera is alive again — reset stale warning
                        if hasattr(self, '_stale_warned'):
                            self._stale_warned.discard(camera_id)
        
        # --- Nearest-frame sync matching ---
        # For each camera, ensure there is at least one frame in the buffer.
        for camera_id, buffer in self.frame_buffers.items():
            if len(buffer) == 0:
                return None  # camera is connected but has no frames yet

        # Choose sync comparison field based on clock correction availability.
        #
        # Clock sync available → use corrected nanosecond timestamps (best accuracy).
        # Clock sync unavailable → use received_at (Mac wall-clock seconds, same reference
        #   for both cameras).  With Windows running at ~15 fps and Mac at ~30 fps,
        #   a Windows frame captured at time T arrives at Mac ~10-20 ms later, while the
        #   nearest Mac frame was captured within ±33 ms of T.  Delta ≈ ≤50 ms, well
        #   inside the 200 ms threshold — completely immune to inter-machine clock skew.
        use_received_at = not self.clock_offsets
        synced_frames = []

        if use_received_at:
            newest_per_cam = {cam: buf[-1].received_at for cam, buf in self.frame_buffers.items()}
            reference = min(newest_per_cam.values())   # anchor = slowest camera (seconds)
            threshold = self._get_sync_threshold_ns() / 1_000_000_000.0  # ns → seconds
            for camera_id, buffer in self.frame_buffers.items():
                best_frame = min(buffer, key=lambda f: abs(f.received_at - reference))
                best_delta = abs(best_frame.received_at - reference)
                if best_delta <= threshold:
                    synced_frames.append(best_frame)
                else:
                    self.stats['sync_failures'] += 1
                    return None
        else:
            # Clock offsets known — compare corrected nanosecond timestamps
            newest_per_cam = {cam: buf[-1].timestamp for cam, buf in self.frame_buffers.items()}
            reference = min(newest_per_cam.values())   # anchor = slowest camera's newest frame (ns)
            threshold = self._get_sync_threshold_ns()  # nanoseconds
            for camera_id, buffer in self.frame_buffers.items():
                best_frame = min(buffer, key=lambda f: abs(f.timestamp - reference))
                best_delta = abs(best_frame.timestamp - reference)
                if best_delta <= threshold:
                    synced_frames.append(best_frame)
                else:
                    self.stats['sync_failures'] += 1
                    return None

        if len(synced_frames) == self.num_cameras:
            # Matched — consume each selected frame and all frames older than it.
            for frame in synced_frames:
                while len(self.frame_buffers[frame.camera_id]) > 0:
                    old_frame = self.frame_buffers[frame.camera_id].popleft()
                    if old_frame == frame:
                        break

            self.stats['frames_synced'] += 1
            return synced_frames

        return None

    def _extract_pose_landmarks(self, results: Dict[str, Any]) -> Optional[list]:
        """
        Normalize pose landmarks from either a raw MediaPipe result or a serialized dict.
        
        Local camera: results['pose'] is a PoseLandmarkerResult object
        Remote camera: results['pose'] is a list of dicts or list of lists
        
        Returns: list of dicts with keys 'x', 'y', 'z', 'visibility', or None
        """
        pose = results.get('pose')
        if not pose:
            packet_landmarks = results.get('packet_landmarks')
            if isinstance(packet_landmarks, list) and len(packet_landmarks) > 0:
                return [
                    {
                        'x': lm.get('x', 0.0),
                        'y': lm.get('y', 0.0),
                        'z': lm.get('z', 0.0),
                        'visibility': lm.get('conf', lm.get('visibility', 1.0))
                    }
                    for lm in packet_landmarks if isinstance(lm, dict)
                ]
            return None

        # Case 1: Raw MediaPipe PoseLandmarkerResult object
        # It has a .pose_landmarks attribute (list of lists of NormalizedLandmark)
        if hasattr(pose, 'pose_landmarks'):
            if pose.pose_landmarks and len(pose.pose_landmarks) > 0:
                # pose_landmarks[0] = first detected person
                return [
                    {'x': lm.x, 'y': lm.y, 'z': lm.z, 'visibility': lm.visibility}
                    for lm in pose.pose_landmarks[0]
                ]
            return None

        # Case 2: Already-serialized list (from remote camera via network)
        if isinstance(pose, list):
            if len(pose) == 0:
                return None
            first = pose[0]
            # Sub-case 2a: list of dicts (JSON deserialized)
            if isinstance(first, dict):
                return pose
            # Sub-case 2b: list of lists [[x,y,z,vis], ...]
            if isinstance(first, (list, tuple)):
                return [
                    {'x': lm[0], 'y': lm[1], 'z': lm[2], 'visibility': lm[3] if len(lm) > 3 else 1.0}
                    for lm in pose
                ]

        # Also handle results['pose_landmarks'] key (alternative serialization)
        pose_lms = results.get('pose_landmarks')
        if pose_lms and isinstance(pose_lms, list) and len(pose_lms) > 0:
            first = pose_lms[0]
            if isinstance(first, dict):
                return pose_lms
            if isinstance(first, list) and len(first) > 0:
                if isinstance(first[0], dict):
                    return first  # pose_landmarks[0] = person 0's landmarks

        packet_landmarks = results.get('packet_landmarks')
        if isinstance(packet_landmarks, list) and len(packet_landmarks) > 0:
            return [
                {
                    'x': lm.get('x', 0.0),
                    'y': lm.get('y', 0.0),
                    'z': lm.get('z', 0.0),
                    'visibility': lm.get('conf', lm.get('visibility', 1.0))
                }
                for lm in packet_landmarks if isinstance(lm, dict)
            ]

        return None

    def _extract_world_landmarks(self, results: Dict[str, Any]) -> Optional[list]:
        """
        Extract MediaPipe pose_world_landmarks (hip-relative, in meters).
        Used as Tier 3 fallback when triangulation and monocular both fail.
        
        Returns: list of dicts with 'x','y','z','visibility', or None
        """
        pose = results.get('pose')

        # Raw MediaPipe result with pose_world_landmarks (local camera, not yet compacted)
        if pose and hasattr(pose, 'pose_world_landmarks'):
            if pose.pose_world_landmarks and len(pose.pose_world_landmarks) > 0:
                return [
                    {'x': lm.x, 'y': lm.y, 'z': lm.z, 'visibility': lm.visibility}
                    for lm in pose.pose_world_landmarks[0]
                ]

        # Serialized world landmarks — compacted Mac frame ('pose_world_landmarks' key)
        # or remote camera ('world_landmarks' key).  Do NOT early-return on missing 'pose'.
        world_lms = results.get('pose_world_landmarks') or results.get('world_landmarks')
        if isinstance(world_lms, list) and len(world_lms) > 0:
            first = world_lms[0]
            if isinstance(first, dict):
                return world_lms
            if isinstance(first, (list, tuple)):
                return [
                    {'x': lm[0], 'y': lm[1], 'z': lm[2],
                     'visibility': lm[3] if len(lm) > 3 else 0.5}
                    for lm in world_lms
                ]
        
        return None

    def _filter_pose_3d(self, pose_3d: Dict[int, Dict[str, Any]], timestamp_ns: int) -> Dict[int, Dict[str, Any]]:
        """Apply 1-Euro filtering to 3D joint positions only (never filter angles)."""
        if not ENABLE_3D_ONE_EURO_FILTER:
            return pose_3d

        t_s = timestamp_ns / 1_000_000_000.0
        filtered = {}

        for lm_id, point in pose_3d.items():
            out = dict(point)
            axis_filters = self.pose_3d_filters[lm_id]

            for axis in ('x', 'y', 'z'):
                value = float(point[axis])
                if axis not in axis_filters:
                    axis_filters[axis] = OneEuroFilter(
                        t0=t_s,
                        x0=value,
                        min_cutoff=FILTER_MIN_CUTOFF,
                        beta=FILTER_BETA,
                        d_cutoff=FILTER_D_CUTOFF
                    )
                else:
                    value = axis_filters[axis](t_s, value)

                out[axis] = float(value)

            filtered[lm_id] = out

        return filtered

    def _apply_world_axis_convention(self, point: Dict[str, Any]) -> Dict[str, Any]:
        """Map triangulated camera-style coordinates to locked world convention."""
        x = float(point['x'])
        y = float(point['y'])
        z = float(point['z'])

        if WORLD_AXIS_TRANSFORM.get('flip_x', False):
            x = -x
        if WORLD_AXIS_TRANSFORM.get('flip_y', False):
            y = -y
        if WORLD_AXIS_TRANSFORM.get('flip_z', False):
            z = -z

        point['x'] = x
        point['y'] = y
        point['z'] = z
        return point

    def _resolve_calibration_camera_id(self, camera_id: str) -> str:
        """Resolve runtime camera IDs to calibration IDs.

        Runtime IDs    Calibration IDs (ArUco)
        -----------    -----------------------
        'local_cam'  → 'local_cam'  (Mac, identity extrinsics)
        'cam_0'      → 'cam_0'      (Windows, NETWORK_CAMERA_ID)

        For legacy default-calibration (pre-ArUco) the same mapping holds
        since create_default_calibration uses 'local_cam' and 'cam_0'.
        """
        if not self.triangulator or not self.triangulator.calibration:
            return camera_id
        cameras = self.triangulator.calibration.cameras
        if camera_id in cameras:
            return camera_id
        # Fallback: if calibration was saved with a different naming convention
        # try well-known aliases so we never silently drop a camera.
        aliases = {
            'local_cam': ('cam_0', 'cam_1', 'camera_0', 'camera_1'),
            'cam_0':     ('cam_1', 'local_cam', 'camera_0'),
        }
        for alias in aliases.get(camera_id, ()):
            if alias in cameras:
                return alias
        return camera_id

    def _compute_pose_3d_kinematics(self, pose_3d: Dict[int, Dict[str, Any]], timestamp_ns: int) -> Dict[str, Any]:
        """Compute linear velocity, acceleration, angles, and angular velocity from filtered 3D pose."""
        frame_result = self.kinematics_engine.process_frame(
            joints_3d=pose_3d,
            timestamp=timestamp_ns,
            compute_derivatives=True,
            min_confidence=KINEMATICS_MIN_POINT_CONFIDENCE,
        )

        joint_velocity = {}
        joint_acceleration = {}
        for lm_id, kinematics in frame_result.get('joint_kinematics', {}).items():
            if kinematics.velocity is not None:
                joint_velocity[lm_id] = {
                    'vx': kinematics.velocity[0],
                    'vy': kinematics.velocity[1],
                    'vz': kinematics.velocity[2],
                    'v': kinematics.velocity_magnitude,
                }
            if kinematics.acceleration is not None:
                joint_acceleration[lm_id] = {
                    'ax': kinematics.acceleration[0],
                    'ay': kinematics.acceleration[1],
                    'az': kinematics.acceleration[2],
                    'a': kinematics.acceleration_magnitude,
                }

        joint_angles = {
            name: data.angle
            for name, data in frame_result.get('angles', {}).items()
        }
        angular_velocity = {
            name: data.angular_velocity
            for name, data in frame_result.get('angles', {}).items()
            if data.angular_velocity is not None
        }

        spine_vector = frame_result.get('spine_vector')
        if spine_vector is None:
            spine_payload = None
        else:
            spine_payload = {'x': spine_vector[0], 'y': spine_vector[1], 'z': spine_vector[2]}

        return {
            'joint_velocity_3d': joint_velocity,
            'joint_acceleration_3d': joint_acceleration,
            'joint_angles_deg': joint_angles,
            'angular_velocity_deg_s': angular_velocity,
            'spine_vector': spine_payload,
            'flat_export': self.kinematics_engine.export_frame_data(frame_result),
        }

    def _get_monocular_fallback(self, lm: Dict[str, Any], camera_id: str) -> Optional[Dict[str, Any]]:
        """
        Estimate 3D position from a single view (Level 1 Fallback).
        Uses MediaPipe's relative 'z' and an assumed distance to subject.
        """
        if not self.triangulator or camera_id not in self.triangulator.calibration.cameras:
            return None
            
        from config import MONOCULAR_SUBJECT_DISTANCE_M
        
        # Get camera parameters
        cam = self.triangulator.calibration.cameras[camera_id]
        fx = cam.intrinsic_matrix[0, 0]
        fy = cam.intrinsic_matrix[1, 1]
        cx = cam.intrinsic_matrix[0, 2]
        cy = cam.intrinsic_matrix[1, 2]
        
        # 1. 2D Normalize -> Pixel -> Camera Space
        # MediaPipe x,y are [0,1]. z is relative to hip (approx 0 centered)
        x_px = lm['x'] * cam.image_size[0]
        y_px = lm['y'] * cam.image_size[1]
        
        # Assumed depth: subject distance + MediaPipe's relative z
        # MediaPipe z is scaled such that it's roughly in meters relative to hip
        z_cam = MONOCULAR_SUBJECT_DISTANCE_M + lm.get('z', 0)
        
        # Back-project to 3D in camera space
        x_cam = (x_px - cx) * z_cam / fx
        y_cam = (y_px - cy) * z_cam / fy
        
        # Transform from camera space to world space: P_world = R.T @ (P_cam - T)
        P_cam = np.array([[x_cam], [y_cam], [z_cam]])
        P_world = cam.rotation.T @ (P_cam - cam.translation)
        
        return {
            'x': float(P_world[0, 0]),
            'y': float(P_world[1, 0]),
            'z': float(P_world[2, 0]),
            'visibility': lm.get('visibility', 0.5) * 0.7, # Lower confidence for fallback
            'method': f'monocular_{camera_id}'
        }

    def get_synced_3d_pose(self, synced_frames: List['FrameData']) -> Optional[Dict[str, Any]]:
        """
        Compute 3D pose from a list of synchronized 2D frames.
        
        Pipeline:
          Tier 1 — Multi-view triangulation (DLT) + inter-camera disagreement
          Tier 2 — Monocular back-projection fallback
          Tier 3 — MediaPipe world-landmarks fallback (hip-relative, scaled)
          Occlusion state machine — VISIBLE → OCCLUDED → PREDICTED
        """
        if not self.triangulator:
            return None
            
        from config import OCCLUSION_FILL_ENABLED
        
        OCCLUSION_MAX_PREDICTED_FRAMES = 15  # ~500ms @ 30fps
        MAX_PREDICT_SPEED_M_S = 8.0
        fused_timestamp_ns = int(np.mean([f.timestamp for f in synced_frames]))
            
        # ── Collect 2D observations for each landmark ──
        landmark_observations = defaultdict(dict)
        landmark_visibilities = defaultdict(dict)
        landmark_raw_data = defaultdict(dict)
        world_landmark_data = defaultdict(dict)  # Tier 3: world landmarks per camera
        
        for frame in synced_frames:
            landmarks = self._extract_pose_landmarks(frame.results)
            if not landmarks:
                continue

            cal_camera_id = self._resolve_calibration_camera_id(frame.camera_id)
            
            # Also extract world landmarks for Tier 3 fallback
            world_lms = self._extract_world_landmarks(frame.results)
                
            for idx, lm in enumerate(landmarks):
                vis = lm.get('visibility', 1.0)
                landmark_raw_data[idx][cal_camera_id] = lm
                if vis >= STEREO_POINT_MIN_INPUT_CONFIDENCE:
                    w, h = 1280, 720
                    if cal_camera_id in self.triangulator.calibration.cameras:
                        image_size = self.triangulator.calibration.cameras[cal_camera_id].image_size
                        if image_size and len(image_size) == 2:
                            w, h = image_size
                    x_px = lm['x'] * w
                    y_px = lm['y'] * h
                    landmark_observations[idx][cal_camera_id] = np.array([x_px, y_px])
                    landmark_visibilities[idx][cal_camera_id] = vis
                    
            # Store world landmarks for Tier 3
            if world_lms:
                for idx, wlm in enumerate(world_lms):
                    world_landmark_data[idx][cal_camera_id] = wlm
        
        # ── Track quality for feedback loop ──
        quality_feedback = {}
        
        # ── Triangulate each landmark (3-tier fallback) ──
        landmarks_3d = {}
        low_reliability_landmarks = []
        uncertainty = {}  # lm_id -> disagreement in meters
        
        for lm_id in set(list(landmark_observations.keys()) + list(world_landmark_data.keys())):
            observations = landmark_observations.get(lm_id, {})
            
            # ── TIER 1: MULTI-VIEW TRIANGULATION + DISAGREEMENT ──
            if len(observations) >= 2:
                point_3d = self.triangulator.triangulate_point(
                    observations, 
                    visibility_weights=landmark_visibilities.get(lm_id, {})
                )
                if point_3d:
                    landmarks_3d[lm_id] = {
                        'x': point_3d.x,
                        'y': point_3d.y,
                        'z': point_3d.z,
                        'visibility': point_3d.confidence,
                        'method': 'triangulated',
                        'views': point_3d.num_views,
                        'reproj_error': point_3d.reprojection_error
                    }
                    if point_3d.confidence < KINEMATICS_MIN_POINT_CONFIDENCE:
                        low_reliability_landmarks.append(lm_id)
                    
                    # ── Inter-camera disagreement (uncertainty) ──
                    # Compute per-camera monocular 3D estimates and measure spread
                    per_cam_estimates = []
                    for cam_id in observations:
                        if cam_id in landmark_raw_data.get(lm_id, {}):
                            mono = self._get_monocular_fallback(
                                landmark_raw_data[lm_id][cam_id], cam_id
                            )
                            if mono:
                                per_cam_estimates.append(
                                    np.array([mono['x'], mono['y'], mono['z']])
                                )
                    if len(per_cam_estimates) >= 2:
                        # Disagreement = max pairwise distance between estimates
                        max_dist = 0.0
                        for i in range(len(per_cam_estimates)):
                            for j in range(i + 1, len(per_cam_estimates)):
                                d = float(np.linalg.norm(
                                    per_cam_estimates[i] - per_cam_estimates[j]
                                ))
                                max_dist = max(max_dist, d)
                        uncertainty[lm_id] = max_dist
                        self._landmark_disagreements[lm_id] = max_dist
                    
                    quality_feedback[lm_id] = point_3d.reprojection_error
                    continue  # Success — skip lower tiers
            
            # ── TIER 2: MONOCULAR FALLBACK ──
            if OCCLUSION_FILL_ENABLED and len(observations) >= 1:
                best_cam = max(
                    landmark_visibilities.get(lm_id, {}),
                    key=lambda k: landmark_visibilities.get(lm_id, {}).get(k, 0),
                    default=None
                )
                if best_cam and landmark_visibilities.get(lm_id, {}).get(best_cam, 0) > 0.5:
                    est_3d = self._get_monocular_fallback(
                        landmark_raw_data[lm_id][best_cam], best_cam
                    )
                    if est_3d:
                        landmarks_3d[lm_id] = est_3d
                        if est_3d.get('visibility', 0.0) < KINEMATICS_MIN_POINT_CONFIDENCE:
                            low_reliability_landmarks.append(lm_id)
                        continue  # Success
            
            # ── TIER 3: WORLD LANDMARKS FALLBACK ──
            # MediaPipe pose_world_landmarks are hip-relative in meters.
            # Average across cameras when available.
            if lm_id in world_landmark_data and world_landmark_data[lm_id]:
                wx, wy, wz, wv = [], [], [], []
                for cam_id, wlm in world_landmark_data[lm_id].items():
                    wx.append(wlm.get('x', 0))
                    wy.append(wlm.get('y', 0))
                    wz.append(wlm.get('z', 0))
                    wv.append(wlm.get('visibility', 0.5))
                if wx:
                    landmarks_3d[lm_id] = {
                        'x': float(np.mean(wx)),
                        'y': float(np.mean(wy)),
                        'z': float(np.mean(wz)),
                        'visibility': float(np.mean(wv)) * 0.5,  # Penalize
                        'method': 'world_landmarks',
                        'views': len(wx)
                    }
                    low_reliability_landmarks.append(lm_id)
                    continue
        
        # ── OCCLUSION STATE MACHINE ──
        # Track per-landmark state: VISIBLE → OCCLUDED → PREDICTED
        all_lm_ids = set(list(landmarks_3d.keys()) + list(self._occlusion_state.keys()))
        for lm_id in all_lm_ids:
            if lm_id in landmarks_3d:
                # Landmark found → VISIBLE
                current_point = landmarks_3d[lm_id]
                prev_point = self._occlusion_last_position.get(lm_id)
                prev_ts = self._occlusion_last_timestamp_ns.get(lm_id)
                if prev_point is not None and prev_ts is not None and fused_timestamp_ns > prev_ts:
                    dt = (fused_timestamp_ns - prev_ts) / 1_000_000_000.0
                    if dt > 1e-6:
                        velocity = np.array([
                            (float(current_point.get('x', 0.0)) - float(prev_point.get('x', 0.0))) / dt,
                            (float(current_point.get('y', 0.0)) - float(prev_point.get('y', 0.0))) / dt,
                            (float(current_point.get('z', 0.0)) - float(prev_point.get('z', 0.0))) / dt,
                        ], dtype=float)
                        speed = float(np.linalg.norm(velocity))
                        if speed > MAX_PREDICT_SPEED_M_S and speed > 1e-6:
                            velocity = velocity * (MAX_PREDICT_SPEED_M_S / speed)
                        self._occlusion_velocity[lm_id] = velocity

                self._occlusion_state[lm_id] = 'VISIBLE'
                self._occlusion_last_position[lm_id] = current_point.copy()
                self._occlusion_last_timestamp_ns[lm_id] = fused_timestamp_ns
                self._occlusion_frames_hidden[lm_id] = 0
            elif lm_id in self._occlusion_last_position:
                # Not found but we have history
                self._occlusion_frames_hidden[lm_id] += 1
                hidden = self._occlusion_frames_hidden[lm_id]
                
                if hidden <= OCCLUSION_MAX_PREDICTED_FRAMES:
                    # Predict from last known state + velocity (or fall back to hold)
                    predicted = self._occlusion_last_position[lm_id].copy()
                    last_ts = self._occlusion_last_timestamp_ns.get(lm_id)
                    dt = 0.0
                    if last_ts is not None and fused_timestamp_ns > last_ts:
                        dt = (fused_timestamp_ns - last_ts) / 1_000_000_000.0

                    velocity = self._occlusion_velocity.get(lm_id)
                    if velocity is not None and dt > 0.0:
                        predicted['x'] = float(predicted.get('x', 0.0) + velocity[0] * dt)
                        predicted['y'] = float(predicted.get('y', 0.0) + velocity[1] * dt)
                        predicted['z'] = float(predicted.get('z', 0.0) + velocity[2] * dt)
                        predicted['method'] = 'predicted_velocity'
                    else:
                        predicted['method'] = 'predicted_hold'

                    predicted['occluded'] = True
                    predicted['visibility'] = max(0.05, float(predicted.get('visibility', 0.5)) * 0.7)
                    landmarks_3d[lm_id] = predicted
                    self._occlusion_state[lm_id] = 'PREDICTED'
                    low_reliability_landmarks.append(lm_id)
                else:
                    # Too long — drop and mark as lost
                    self._occlusion_state[lm_id] = 'OCCLUDED'

        if not landmarks_3d:
            return None
            
        # ── Feedback loop ──
        self.frame_count += 1
        if FEEDBACK_ENABLED and self.feedback_socket and self.frame_count % FEEDBACK_INTERVAL_FRAMES == 0:
            self._send_quality_feedback(quality_feedback)
            
        # ── World axis convention ──
        for lm_id in list(landmarks_3d.keys()):
            landmarks_3d[lm_id] = self._apply_world_axis_convention(landmarks_3d[lm_id])

        landmarks_3d = self._filter_pose_3d(landmarks_3d, fused_timestamp_ns)
        kinematics = self._compute_pose_3d_kinematics(landmarks_3d, fused_timestamp_ns)

        # ── Optional standalone module outputs (additive, non-breaking) ──
        # Build per-camera 3D estimates for uncertainty/fusion modules
        per_camera_points = defaultdict(list)
        for lm_id in landmarks_3d.keys():
            raw_map = landmark_raw_data.get(lm_id, {})
            for cam_id, raw_lm in raw_map.items():
                mono = self._get_monocular_fallback(raw_lm, cam_id)
                if mono:
                    per_camera_points[lm_id].append(np.array([
                        mono.get('x', 0.0), mono.get('y', 0.0), mono.get('z', 0.0)
                    ], dtype=float))

        triangulated_only = {
            lm_id: pt for lm_id, pt in landmarks_3d.items()
            if pt.get('method') == 'triangulated'
        }
        monocular_only = {
            lm_id: pt for lm_id, pt in landmarks_3d.items()
            if str(pt.get('method', '')).startswith('monocular_')
        }
        world_only = {
            lm_id: pt for lm_id, pt in landmarks_3d.items()
            if pt.get('method') == 'world_landmarks'
        }

        fusion_output = self.occlusion_fusion_engine.fuse_frame(
            triangulated=triangulated_only,
            monocular=monocular_only,
            world_landmarks=world_only,
            per_camera_estimates=per_camera_points,
            timestamp_ns=fused_timestamp_ns
        )

        reproj_errors = {
            lm_id: float(pt.get('reproj_error', 0.0))
            for lm_id, pt in landmarks_3d.items()
            if isinstance(pt, dict)
        }

        # ── Build occlusion summary ──
        occlusion_summary = {}
        for lm_id in landmarks_3d:
            occlusion_summary[lm_id] = self._occlusion_state.get(lm_id, 'VISIBLE')

        uncertainty_detailed = self.uncertainty_estimator.estimate_frame(
            landmarks_3d=landmarks_3d,
            per_camera_points=per_camera_points,
            visibility_scores=landmark_visibilities,
            occlusion_states=occlusion_summary,
            reproj_errors=reproj_errors
        )
        self.error_metrics_calculator.add_frame(uncertainty_detailed)
        self.error_metrics_calculator.add_consistency_score(
            self.occlusion_fusion_engine.get_consistency_score()
        )
        uncertainty_summary = self.error_metrics_calculator.get_summary()

        advanced_state = self.advanced_kinematics_engine.process_frame(
            landmarks_3d, fused_timestamp_ns,
            min_confidence=KINEMATICS_MIN_POINT_CONFIDENCE
        )
        advanced_kinematics = self.advanced_kinematics_engine.export_state_dict(advanced_state)

        return {
            'pose_3d': landmarks_3d,
            'kinematics_3d': kinematics,
            'low_reliability_landmarks': sorted(set(low_reliability_landmarks)),
            'timestamp_ns': fused_timestamp_ns,
            'uncertainty': uncertainty,
            'occlusion_states': occlusion_summary,
            'fusion': fusion_output,
            'uncertainty_detailed': {
                lm_id: {
                    'combined_uncertainty_m': u.combined_uncertainty_m,
                    'confidence': u.confidence,
                    'reproj_error_px': u.reproj_error_px,
                    'inter_camera_disagreement_m': u.inter_camera_disagreement_m,
                    'method': u.method
                }
                for lm_id, u in uncertainty_detailed.items()
            },
            'uncertainty_summary': uncertainty_summary,
            'advanced_kinematics': advanced_kinematics,
            'dashboard_status': self.dashboard_monitor.get_status()
        }

    def get_dashboard_status(self) -> Dict[str, Any]:
        """Expose consolidated dashboard monitoring status."""
        return self.dashboard_monitor.get_status()

    def _send_quality_feedback(self, quality_hints: Dict[int, float]):
        """Broadcast quality metrics to all cameras."""
        if not self.feedback_socket:
            return

        compute_hints = self._build_compute_hints()
            
        msg = {
            'type': 'quality_feedback',
            'timestamp': time.time_ns(),
            'hints': quality_hints,  # {lm_id: reproj_error}
            'compute_hints': compute_hints
        }
        
        try:
            payload = msgpack.packb(msg)
            self.feedback_socket.send(payload)
        except Exception as e:
            print(f"[MasterCoordinator] Feedback send error: {e}")

    def _build_compute_hints(self) -> Dict[str, Any]:
        """
        Build lightweight scheduling hints for camera nodes based on bottlenecks
        and recent per-camera inference reports.
        """
        stats = self.get_latency_stats()
        stage_means = {
            stage: s.get('mean_ms', 0.0)
            for stage, s in stats.items()
            if isinstance(s, dict)
        }
        if not stage_means:
            return {
                'suggested_action': 'none',
                'target_stage': 'unknown',
                'reason': 'insufficient_latency_samples'
            }

        target_stage = max(stage_means, key=lambda k: stage_means[k])
        target_ms = float(stage_means.get(target_stage, 0.0))

        # Conservative policy: only suggest downshift on clearly slow stages.
        if target_stage == 'detection' and target_ms > 35.0:
            action = 'reduce_detector_load'
        elif target_stage == 'network' and target_ms > 20.0:
            action = 'reduce_network_payload'
        elif target_stage == 'triangulation' and target_ms > 12.0:
            action = 'reduce_3d_load'
        else:
            action = 'none'

        camera_compute = {}
        for camera_id, report in self._gpu_reports.items():
            if not isinstance(report, dict):
                continue
            inf = report.get('inference_times_ms', {}) or {}
            camera_compute[camera_id] = {
                'pose_ms': float(inf.get('pose', 0.0)),
                'face_ms': float(inf.get('face', 0.0)),
                'hand_ms': float(inf.get('hand', 0.0)),
                'total_ms': float(inf.get('total', 0.0)),
                'device': report.get('device', 'unknown')
            }

        return {
            'suggested_action': action,
            'target_stage': target_stage,
            'target_stage_mean_ms': round(target_ms, 2),
            'camera_compute': camera_compute,
            'generated_at_ns': time.time_ns()
        }
    
    def discover_cameras(self, timeout: float = 5.0) -> List[CameraInfo]:
        """
        Wait for cameras to be discovered.
        
        Args:
            timeout: Maximum time to wait (seconds)
            
        Returns:
            List of discovered cameras
        """
        if not self.running:
            self.start()
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            if len(self.discovered_cameras) >= self.num_cameras:
                break
            time.sleep(0.1)
        
        return list(self.discovered_cameras.values())
    
    def get_stats(self) -> Dict[str, Any]:
        """Get coordinator statistics."""
        return {
            'num_cameras_discovered': len(self.discovered_cameras),
            'cameras': list(self.discovered_cameras.keys()),
            'frames_received': dict(self.stats['frames_received']),
            'frames_synced': self.stats['frames_synced'],
            'sync_failures': self.stats['sync_failures'],
            'buffer_sizes': {
                cam_id: len(buffer) 
                for cam_id, buffer in self.frame_buffers.items()
            }
        }
    
    def clear_buffers(self):
        """Clear all frame buffers."""
        for buffer in self.frame_buffers.values():
            buffer.clear()


if __name__ == "__main__":
    # Test coordinator
    print("Starting Master Coordinator test...")
    coordinator = MasterCoordinator(num_cameras=2)
    coordinator.start()
    
    try:
        # Discover cameras
        print("Discovering cameras...")
        cameras = coordinator.discover_cameras(timeout=10.0)
        print(f"Discovered {len(cameras)} cameras: {[c.camera_id for c in cameras]}")
        
        # Monitor for synced frames
        print("Monitoring synchronized frames...")
        for _ in range(100):
            batch = coordinator.get_synchronized_batch()
            if batch:
                print(f"Synced batch: {[f.camera_id for f in batch]}")
            time.sleep(0.1)
        
        # Print stats
        print("\nStatistics:")
        print(json.dumps(coordinator.get_stats(), indent=2))
        
    except KeyboardInterrupt:
        pass
    finally:
        coordinator.stop()
