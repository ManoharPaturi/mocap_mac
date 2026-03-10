import json
import uuid
import time
import threading
import queue
import csv
import io
from typing import Optional, Dict, List, Any, Tuple
from datetime import datetime
from config import DB_TYPE, DB_PATH, POSTGRES_CONNECTION
from src.calculations import Calculations

# Conditional imports
if DB_TYPE == 'postgres':
    import psycopg2
    import psycopg2.extras
else:
    import sqlite3

# ... imports remain same
from config import DB_TYPE, DB_PATH, POSTGRES_CONNECTION
from src.calculations import Calculations

# Conditional imports
if DB_TYPE == 'postgres':
    import psycopg2
    import psycopg2.extras
else:
    import sqlite3

class MocapDB:
    def __init__(self):
        self.db_type = DB_TYPE
        self.queue = queue.Queue(maxsize=120)
        self.running = False
        self.session_id = None
        self.current_table = None # Track current table name
        self.thread = None
        self._dropped_queue_items = 0
        self._save_trace_enabled = True
        self._save_trace_every = 30
        self._save_trace_count = 0
        self._init_db()

    def _enqueue_data(self, data: Dict[str, Any]):
        """Bounded enqueue with drop-oldest policy to prevent queue-memory blowup."""
        try:
            self.queue.put_nowait(data)
            return
        except queue.Full:
            pass

        # Drop oldest stale item and retry once.
        try:
            self.queue.get_nowait()
            self._dropped_queue_items += 1
        except queue.Empty:
            pass

        try:
            self.queue.put_nowait(data)
        except queue.Full:
            self._dropped_queue_items += 1

    def _get_connection(self):
        """Get database connection based on type."""
        if self.db_type == 'postgres':
            return psycopg2.connect(**POSTGRES_CONNECTION)
        else:
            return sqlite3.connect(DB_PATH)

    def _init_db(self):
        """Initialize database schema."""
        conn = self._get_connection()
        c = conn.cursor()
        
        # Sessions Master Table
        # id: UUID, start_time: ISO timestamp, table_name: storage table
        schema = '''CREATE TABLE IF NOT EXISTS sessions
                 (id TEXT PRIMARY KEY,
                  start_time TEXT,
                  table_name TEXT,
                  recording_mode TEXT DEFAULT 'stereo')'''
        c.execute(schema)
        
        # Schema Migration: Add table_name if missing
        try:
            if self.db_type == 'sqlite':
                c.execute("ALTER TABLE sessions ADD COLUMN table_name TEXT")
            else:
                c.execute("ALTER TABLE sessions ADD COLUMN IF NOT EXISTS table_name TEXT")
        except: pass

        # Schema Migration: Add recording_mode if missing
        try:
            if self.db_type == 'sqlite':
                c.execute("ALTER TABLE sessions ADD COLUMN recording_mode TEXT DEFAULT 'stereo'")
            else:
                c.execute("ALTER TABLE sessions ADD COLUMN IF NOT EXISTS recording_mode TEXT DEFAULT 'stereo'")
        except:
            pass

        # Layered self-aware tables (global across sessions)
        c.execute('''CREATE TABLE IF NOT EXISTS frames
                     (id INTEGER PRIMARY KEY AUTOINCREMENT,
                      session_id TEXT,
                      timestamp_ms REAL,
                      frame_index INTEGER,
                      num_cameras INTEGER,
                      created_at TEXT)''')

        c.execute('''CREATE TABLE IF NOT EXISTS raw_landmarks_2d
                     (id INTEGER PRIMARY KEY AUTOINCREMENT,
                      session_id TEXT,
                      frame_id INTEGER,
                      camera_id TEXT,
                      landmark_id INTEGER,
                      x REAL,
                      y REAL,
                      z REAL,
                      visibility REAL,
                      timestamp_ms REAL,
                      source TEXT DEFAULT 'realtime')''')

        c.execute('''CREATE TABLE IF NOT EXISTS joints_3d
                     (id INTEGER PRIMARY KEY AUTOINCREMENT,
                      session_id TEXT,
                      frame_id INTEGER,
                      joint_id INTEGER,
                      x REAL,
                      y REAL,
                      z REAL,
                      confidence REAL,
                      reprojection_error_px REAL,
                      timestamp_ms REAL,
                      triangulation_version TEXT,
                      algorithm TEXT,
                      source TEXT DEFAULT 'stereo')''')

        c.execute('''CREATE TABLE IF NOT EXISTS kinematics_3d
                     (id INTEGER PRIMARY KEY AUTOINCREMENT,
                      session_id TEXT,
                      frame_id INTEGER,
                      angle_name TEXT,
                      angle_deg REAL,
                      timestamp_ms REAL,
                      smoothing_version TEXT,
                      triangulation_version TEXT,
                      source TEXT DEFAULT 'realtime',
                      confidence REAL,
                      raw_angle_degrees REAL)''')

        c.execute('''CREATE TABLE IF NOT EXISTS kinematics_2d
                     (id INTEGER PRIMARY KEY AUTOINCREMENT,
                      session_id TEXT,
                      frame_id INTEGER,
                      camera_id TEXT,
                      angle_name TEXT,
                      angle_deg REAL,
                      timestamp_ms REAL,
                      smoothing_version TEXT,
                      source TEXT DEFAULT 'single')''')

        c.execute('''CREATE TABLE IF NOT EXISTS velocities
                     (id INTEGER PRIMARY KEY AUTOINCREMENT,
                      session_id TEXT,
                      frame_id INTEGER,
                      metric_name TEXT,
                      value REAL,
                      timestamp_ms REAL,
                      source TEXT DEFAULT 'realtime')''')

        c.execute('''CREATE TABLE IF NOT EXISTS accelerations
                     (id INTEGER PRIMARY KEY AUTOINCREMENT,
                      session_id TEXT,
                      frame_id INTEGER,
                      metric_name TEXT,
                      value REAL,
                      timestamp_ms REAL,
                      source TEXT DEFAULT 'realtime')''')

        c.execute('''CREATE TABLE IF NOT EXISTS validation_runs
                     (id INTEGER PRIMARY KEY AUTOINCREMENT,
                      session_id TEXT,
                      created_at TEXT,
                      num_frames INTEGER,
                      num_angles INTEGER,
                      pass_rate REAL,
                      mean_error REAL,
                      max_error REAL,
                      threshold REAL)''')

        c.execute('''CREATE TABLE IF NOT EXISTS validation_angle_comparison
                     (id INTEGER PRIMARY KEY AUTOINCREMENT,
                      validation_run_id INTEGER,
                      session_id TEXT,
                      frame_id INTEGER,
                      angle_name TEXT,
                      realtime_angle REAL,
                      offline_angle REAL,
                      error_deg REAL,
                      pass_flag INTEGER)''')

        c.execute('''CREATE TABLE IF NOT EXISTS validation_artifacts
                     (id INTEGER PRIMARY KEY AUTOINCREMENT,
                      validation_run_id INTEGER,
                      session_id TEXT,
                      artifact_type TEXT,
                      file_path TEXT,
                      metadata_json TEXT,
                      created_at TEXT)''')

        c.execute('''CREATE TABLE IF NOT EXISTS processing_metadata
                 (id INTEGER PRIMARY KEY AUTOINCREMENT,
                  session_id TEXT UNIQUE,
                  created_at TEXT,
                  updated_at TEXT,
                  sync_threshold_ms REAL,
                  sync_strategy TEXT,
                  clock_sync_enabled INTEGER,
                  clock_sync_interval_sec REAL,
                  triangulation_method TEXT,
                  triangulation_version TEXT,
                  smoothing_version TEXT,
                  calibration_id TEXT,
                  fps_target REAL,
                  fps_achieved REAL,
                  filter_min_cutoff REAL,
                  filter_beta REAL,
                  filter_d_cutoff REAL,
                  metadata_json TEXT)''')
            
        conn.commit()
        conn.close()
        print(f"Database initialized: {self.db_type.upper()}")

    def start_recording(self, recording_mode: str = 'stereo'):
        """Start a new recording session."""
        self.session_id = str(uuid.uuid4())
        timestamp = datetime.now()
        start_time = timestamp.isoformat()
        
        # Create Table Name: session_YYYYMMDD_HHMMSS
        date_str = timestamp.strftime("%Y%m%d_%H%M%S")
        self.current_table = f"session_{date_str}"
        
        conn = self._get_connection()
        c = conn.cursor()
        
        # 1. Create Dynamic Table for this session (Expanded for Multi-Cam)
        create_table_sql = f'''CREATE TABLE IF NOT EXISTS {self.current_table}
                             (id {("SERIAL PRIMARY KEY" if self.db_type == 'postgres' else "INTEGER PRIMARY KEY AUTOINCREMENT")},
                              timestamp REAL,
                              -- PC1 (Local)
                              pose_data TEXT,
                              face_data TEXT,
                              hand_data TEXT,
                              derived_data TEXT,
                              -- PC2 (Remote)
                              pc2_pose_data TEXT,
                              pc2_face_data TEXT,
                              pc2_hand_data TEXT,
                              pc2_derived_data TEXT,
                              -- 3D / Combined
                              pose_3d_data TEXT,
                              combined_derived_data TEXT,
                              kinematics_flat_data TEXT,
                              confidence_data TEXT)'''
        c.execute(create_table_sql)
        
        # 2. Register Session
        insert_sql = "INSERT INTO sessions (id, start_time, table_name, recording_mode) VALUES (%s, %s, %s, %s)" if self.db_type == 'postgres' else \
                 "INSERT INTO sessions (id, start_time, table_name, recording_mode) VALUES (?, ?, ?, ?)"
        c.execute(insert_sql, (self.session_id, start_time, self.current_table, recording_mode))
        
        conn.commit()
        conn.close()
        
        self.running = True
        self.thread = threading.Thread(target=self._worker_loop)
        self.thread.daemon = True
        self.thread.start()
        print(f"Recording Started: {self.session_id} -> Table: {self.current_table}")
        return self.session_id

    def stop_recording(self):
        """Stop the current recording session."""
        self.running = False
        if self.thread:
            self.thread.join()
        saved_id = self.session_id
        self.session_id = None
        self.current_table = None
        print("Recording Stopped.")
        return saved_id

    def save_frame_metadata(self, timestamp_ms: float, frame_index: int, num_cameras: int) -> Optional[int]:
        """Layer 0: Save frame timeline metadata and return frame_id."""
        if not self.session_id:
            return None
        conn = self._get_connection()
        c = conn.cursor()
        now_iso = datetime.now().isoformat()
        values = (self.session_id, float(timestamp_ms), int(frame_index), int(num_cameras), now_iso)
        if self.db_type == 'postgres':
            c.execute(
                "INSERT INTO frames (session_id, timestamp_ms, frame_index, num_cameras, created_at) "
                "VALUES (%s, %s, %s, %s, %s) RETURNING id", values
            )
            frame_id = c.fetchone()[0]
        else:
            c.execute(
                "INSERT INTO frames (session_id, timestamp_ms, frame_index, num_cameras, created_at) "
                "VALUES (?, ?, ?, ?, ?)", values
            )
            frame_id = c.lastrowid
        conn.commit()
        conn.close()
        return frame_id

    def save_raw_landmarks(self, frame_id: int, camera_id: str, landmarks_list: List[Dict[str, Any]],
                           timestamp_ms: float, source: str = 'realtime'):
        """Layer 1: Save immutable raw 2D landmarks for one camera."""
        if not self.session_id or frame_id is None:
            return
        if not isinstance(landmarks_list, list):
            return
        conn = self._get_connection()
        c = conn.cursor()
        for lm_id, lm in enumerate(landmarks_list):
            if not isinstance(lm, dict):
                continue
            x = float(lm.get('x', 0.0))
            y = float(lm.get('y', 0.0))
            z = float(lm.get('z', 0.0))
            visibility = float(lm.get('v', lm.get('visibility', lm.get('conf', 0.0))))
            row = (self.session_id, frame_id, camera_id, lm_id, x, y, z, visibility, float(timestamp_ms), source)
            if self.db_type == 'postgres':
                c.execute(
                    "INSERT INTO raw_landmarks_2d (session_id, frame_id, camera_id, landmark_id, x, y, z, visibility, timestamp_ms, source) "
                    "VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s)", row
                )
            else:
                c.execute(
                    "INSERT INTO raw_landmarks_2d (session_id, frame_id, camera_id, landmark_id, x, y, z, visibility, timestamp_ms, source) "
                    "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)", row
                )
        conn.commit()
        conn.close()

    def save_joints_3d(self, frame_id: int, joints_dict: Dict[Any, Dict[str, Any]], timestamp_ms: float,
                       triangulation_version: str = '1.0', algorithm: str = 'opencv_triangulate',
                       source: str = 'stereo'):
        """Layer 2: Save triangulated 3D joints with confidence and reprojection metadata."""
        if not self.session_id or frame_id is None or not isinstance(joints_dict, dict):
            return
        conn = self._get_connection()
        c = conn.cursor()
        for joint_id, item in joints_dict.items():
            if not isinstance(item, dict):
                continue
            row = (
                self.session_id, frame_id, int(joint_id),
                float(item.get('x', 0.0)), float(item.get('y', 0.0)), float(item.get('z', 0.0)),
                float(item.get('confidence', item.get('visibility', 0.0))),
                item.get('reprojection_error_px', item.get('reproj_error')),
                float(timestamp_ms), triangulation_version, algorithm, source
            )
            if self.db_type == 'postgres':
                c.execute(
                    "INSERT INTO joints_3d (session_id, frame_id, joint_id, x, y, z, confidence, reprojection_error_px, timestamp_ms, triangulation_version, algorithm, source) "
                    "VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)", row
                )
            else:
                c.execute(
                    "INSERT INTO joints_3d (session_id, frame_id, joint_id, x, y, z, confidence, reprojection_error_px, timestamp_ms, triangulation_version, algorithm, source) "
                    "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)", row
                )
        conn.commit()
        conn.close()

    def save_kinematics_3d(self, frame_id: int, angles_dict: Dict[str, Any], timestamp_ms: float,
                           smoothing_version: str = '1.0', triangulation_version: str = '1.0',
                           source: str = 'realtime'):
        """Layer 3: Save derived 3D kinematics with algorithm metadata."""
        if not self.session_id or frame_id is None or not isinstance(angles_dict, dict):
            return
        conn = self._get_connection()
        c = conn.cursor()
        for angle_name, value in angles_dict.items():
            try:
                angle_val = float(value)
            except Exception:
                continue
            row = (
                self.session_id, frame_id, angle_name, angle_val, float(timestamp_ms),
                smoothing_version, triangulation_version, source, None, angle_val
            )
            if self.db_type == 'postgres':
                c.execute(
                    "INSERT INTO kinematics_3d (session_id, frame_id, angle_name, angle_deg, timestamp_ms, smoothing_version, triangulation_version, source, confidence, raw_angle_degrees) "
                    "VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s)", row
                )
            else:
                c.execute(
                    "INSERT INTO kinematics_3d (session_id, frame_id, angle_name, angle_deg, timestamp_ms, smoothing_version, triangulation_version, source, confidence, raw_angle_degrees) "
                    "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)", row
                )
        conn.commit()
        conn.close()

    def save_validation_run(self, num_frames: int, num_angles: int, pass_rate: float,
                            mean_error: float, max_error: float, threshold: float,
                            session_id: Optional[str] = None) -> Optional[int]:
        """Layer 4: Save a validation run summary and return validation_run_id."""
        sid = session_id or self.session_id
        if not sid:
            return None
        conn = self._get_connection()
        c = conn.cursor()
        now_iso = datetime.now().isoformat()
        values = (sid, now_iso, int(num_frames), int(num_angles), float(pass_rate), float(mean_error), float(max_error), float(threshold))
        if self.db_type == 'postgres':
            c.execute(
                "INSERT INTO validation_runs (session_id, created_at, num_frames, num_angles, pass_rate, mean_error, max_error, threshold) "
                "VALUES (%s, %s, %s, %s, %s, %s, %s, %s) RETURNING id", values
            )
            run_id = c.fetchone()[0]
        else:
            c.execute(
                "INSERT INTO validation_runs (session_id, created_at, num_frames, num_angles, pass_rate, mean_error, max_error, threshold) "
                "VALUES (?, ?, ?, ?, ?, ?, ?, ?)", values
            )
            run_id = c.lastrowid
        conn.commit()
        conn.close()
        return run_id

    def save_processing_metadata(self,
                                 sync_threshold_ms: float,
                                 sync_strategy: str,
                                 clock_sync_enabled: bool,
                                 clock_sync_interval_sec: float,
                                 triangulation_method: str,
                                 triangulation_version: str,
                                 smoothing_version: str,
                                 calibration_id: Optional[str],
                                 fps_target: float,
                                 fps_achieved: Optional[float] = None,
                                 filter_min_cutoff: Optional[float] = None,
                                 filter_beta: Optional[float] = None,
                                 filter_d_cutoff: Optional[float] = None,
                                 metadata: Optional[Dict[str, Any]] = None,
                                 session_id: Optional[str] = None):
        """Save or update per-session processing environment metadata."""
        sid = session_id or self.session_id
        if not sid:
            return

        now_iso = datetime.now().isoformat()
        metadata_json = json.dumps(metadata or {})

        conn = self._get_connection()
        c = conn.cursor()

        values = (
            sid,
            now_iso,
            now_iso,
            float(sync_threshold_ms),
            str(sync_strategy),
            1 if clock_sync_enabled else 0,
            float(clock_sync_interval_sec),
            str(triangulation_method),
            str(triangulation_version),
            str(smoothing_version),
            calibration_id,
            float(fps_target),
            float(fps_achieved) if fps_achieved is not None else None,
            float(filter_min_cutoff) if filter_min_cutoff is not None else None,
            float(filter_beta) if filter_beta is not None else None,
            float(filter_d_cutoff) if filter_d_cutoff is not None else None,
            metadata_json,
        )

        if self.db_type == 'postgres':
            c.execute(
                """
                INSERT INTO processing_metadata (
                    session_id, created_at, updated_at, sync_threshold_ms, sync_strategy,
                    clock_sync_enabled, clock_sync_interval_sec, triangulation_method,
                    triangulation_version, smoothing_version, calibration_id,
                    fps_target, fps_achieved, filter_min_cutoff, filter_beta,
                    filter_d_cutoff, metadata_json
                ) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)
                ON CONFLICT (session_id) DO UPDATE SET
                    updated_at = EXCLUDED.updated_at,
                    sync_threshold_ms = EXCLUDED.sync_threshold_ms,
                    sync_strategy = EXCLUDED.sync_strategy,
                    clock_sync_enabled = EXCLUDED.clock_sync_enabled,
                    clock_sync_interval_sec = EXCLUDED.clock_sync_interval_sec,
                    triangulation_method = EXCLUDED.triangulation_method,
                    triangulation_version = EXCLUDED.triangulation_version,
                    smoothing_version = EXCLUDED.smoothing_version,
                    calibration_id = EXCLUDED.calibration_id,
                    fps_target = EXCLUDED.fps_target,
                    fps_achieved = EXCLUDED.fps_achieved,
                    filter_min_cutoff = EXCLUDED.filter_min_cutoff,
                    filter_beta = EXCLUDED.filter_beta,
                    filter_d_cutoff = EXCLUDED.filter_d_cutoff,
                    metadata_json = EXCLUDED.metadata_json
                """,
                values
            )
        else:
            c.execute(
                """
                INSERT INTO processing_metadata (
                    session_id, created_at, updated_at, sync_threshold_ms, sync_strategy,
                    clock_sync_enabled, clock_sync_interval_sec, triangulation_method,
                    triangulation_version, smoothing_version, calibration_id,
                    fps_target, fps_achieved, filter_min_cutoff, filter_beta,
                    filter_d_cutoff, metadata_json
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(session_id) DO UPDATE SET
                    updated_at = excluded.updated_at,
                    sync_threshold_ms = excluded.sync_threshold_ms,
                    sync_strategy = excluded.sync_strategy,
                    clock_sync_enabled = excluded.clock_sync_enabled,
                    clock_sync_interval_sec = excluded.clock_sync_interval_sec,
                    triangulation_method = excluded.triangulation_method,
                    triangulation_version = excluded.triangulation_version,
                    smoothing_version = excluded.smoothing_version,
                    calibration_id = excluded.calibration_id,
                    fps_target = excluded.fps_target,
                    fps_achieved = excluded.fps_achieved,
                    filter_min_cutoff = excluded.filter_min_cutoff,
                    filter_beta = excluded.filter_beta,
                    filter_d_cutoff = excluded.filter_d_cutoff,
                    metadata_json = excluded.metadata_json
                """,
                values
            )

        conn.commit()
        conn.close()

    def get_session_frames(self, session_id: str) -> List[Tuple[int, float, int]]:
        """Return [(frame_id, timestamp_ms, frame_index), ...] ordered by frame_index."""
        conn = self._get_connection()
        c = conn.cursor()
        placeholder = '%s' if self.db_type == 'postgres' else '?'
        c.execute(
            f"SELECT id, timestamp_ms, frame_index FROM frames WHERE session_id = {placeholder} ORDER BY frame_index ASC",
            (session_id,)
        )
        rows = c.fetchall()
        conn.close()
        return rows or []

    def get_joints_3d_for_frame(self, frame_id: int) -> Dict[int, Dict[str, Any]]:
        """Return per-joint 3D dict for one frame."""
        conn = self._get_connection()
        c = conn.cursor()
        placeholder = '%s' if self.db_type == 'postgres' else '?'
        c.execute(
            f"SELECT joint_id, x, y, z, confidence, reprojection_error_px, triangulation_version, algorithm, source "
            f"FROM joints_3d WHERE frame_id = {placeholder} ORDER BY joint_id ASC",
            (frame_id,)
        )
        rows = c.fetchall()
        conn.close()
        result = {}
        for r in rows or []:
            result[int(r[0])] = {
                'x': float(r[1]), 'y': float(r[2]), 'z': float(r[3]),
                'confidence': float(r[4]) if r[4] is not None else 0.0,
                'reprojection_error_px': float(r[5]) if r[5] is not None else None,
                'triangulation_version': r[6], 'algorithm': r[7], 'source': r[8]
            }
        return result

    def get_kinematics_3d_for_frame(self, frame_id: int) -> Dict[str, Dict[str, Any]]:
        """Return angle_name -> {'angle': value, 'confidence': ..., ...}."""
        conn = self._get_connection()
        c = conn.cursor()
        placeholder = '%s' if self.db_type == 'postgres' else '?'
        c.execute(
            f"SELECT angle_name, angle_deg, confidence, smoothing_version, triangulation_version, source "
            f"FROM kinematics_3d WHERE frame_id = {placeholder}",
            (frame_id,)
        )
        rows = c.fetchall()
        conn.close()
        out = {}
        for r in rows or []:
            out[str(r[0])] = {
                'angle': float(r[1]) if r[1] is not None else 0.0,
                'confidence': float(r[2]) if r[2] is not None else None,
                'smoothing_version': r[3],
                'triangulation_version': r[4],
                'source': r[5],
            }
        return out

    def get_raw_landmarks_for_frame(self, frame_id: int, camera_id: Optional[str] = None) -> List[Tuple[int, float, float, float, float]]:
        """Return list of (landmark_id, x, y, z, visibility), optionally filtered by camera_id."""
        conn = self._get_connection()
        c = conn.cursor()
        placeholder = '%s' if self.db_type == 'postgres' else '?'
        if camera_id:
            c.execute(
                f"SELECT landmark_id, x, y, z, visibility FROM raw_landmarks_2d WHERE frame_id = {placeholder} AND camera_id = {placeholder} ORDER BY landmark_id ASC",
                (frame_id, camera_id)
            )
        else:
            c.execute(
                f"SELECT landmark_id, x, y, z, visibility FROM raw_landmarks_2d WHERE frame_id = {placeholder} ORDER BY landmark_id ASC",
                (frame_id,)
            )
        rows = c.fetchall()
        conn.close()
        return rows or []

    def save_frame(self, results):
        """Queue a single-camera frame (legacy/PC1-only mode)."""
        if not self.running or not self.session_id:
            return
            
        # Serialize raw landmarks
        pose_data = self._serialize_landmarks(results.get('pose'), 'pose_landmarks')
        face_data = self._serialize_landmarks(results.get('face'), 'face_landmarks')
        hand_data = self._serialize_landmarks(results.get('hand'), 'hand_landmarks')
        
        # Calculate Derived Metrics
        derived_data = []
        for p_idx, person_pose in enumerate(pose_data):
            p_metrics = Calculations.get_body_metrics(person_pose)
            if p_idx < len(face_data):
                f_metrics = Calculations.get_face_metrics(face_data[p_idx])
                p_metrics.update(f_metrics)
            derived_data.append(p_metrics)
            
        data = {
            'type': 'single',
            'timestamp': time.time(),
            'pose': pose_data,
            'face': face_data,
            'hand': hand_data,
            'derived': derived_data
        }
        self._enqueue_data(data)

    def save_synced_frame(self, timestamp, pc1_results, pc2_results, pose_3d):
        """Queue a synchronized multi-camera frame."""
        if not self.running or not self.session_id:
            return

        def _extract_pc2_pose_data(pc2_res):
            if not isinstance(pc2_res, dict):
                return []

            # 1. Try to get the data from any of the possible keys
            raw_pc2_data = pc2_res.get('pose_landmarks') or pc2_res.get('landmarks') or pc2_res.get('packet_landmarks') or []

            # 2. Fix the list nesting for compact payload
            if raw_pc2_data and isinstance(raw_pc2_data, list) and len(raw_pc2_data) > 0 and isinstance(raw_pc2_data[0], dict):
                pc2_data = [raw_pc2_data]
            else:
                pc2_data = raw_pc2_data

            return pc2_data if isinstance(pc2_data, list) else []

        def _normalize_compact_pose_payload(results):
            """Return pose landmarks in list-of-people shape from compact network payloads."""
            if not isinstance(results, dict):
                return None

            packet = results.get('packet_landmarks')
            if packet is None:
                packet = results.get('landmarks')

            # Defensive support for wrapped payloads.
            if packet is None:
                inner = results.get('results')
                if isinstance(inner, dict):
                    packet = inner.get('packet_landmarks') or inner.get('landmarks')

            if not isinstance(packet, list) or len(packet) == 0:
                return None

            src = packet[0] if isinstance(packet[0], list) else packet
            person = [
                {
                    'x': round(float(lm.get('x', 0.0)), 5),
                    'y': round(float(lm.get('y', 0.0)), 5),
                    'z': round(float(lm.get('z', 0.0)), 5),
                    'v': round(float(lm.get('conf', lm.get('visibility', 1.0))), 5)
                }
                for lm in src if isinstance(lm, dict)
            ]
            return [person] if person else None

        def _has_compact_pose_payload(results) -> bool:
            normalized = _normalize_compact_pose_payload(results)
            return bool(normalized and len(normalized[0]) > 0)

        def process_results(results, is_pc2=False):
            if not results: return [], [], [], []
            if isinstance(results, dict):
                # Defensive unwrap for transport wrappers.
                wrapped = results.get('results')
                if isinstance(wrapped, dict):
                    merged = dict(wrapped)
                    for k in ('landmarks', 'packet_landmarks', 'pose_landmarks', 'pose', 'face_landmarks', 'hand_landmarks'):
                        if k in results and k not in merged:
                            merged[k] = results.get(k)
                    results = merged

                if is_pc2:
                    pc2_data = _extract_pc2_pose_data(results)
                    if pc2_data:
                        patched = dict(results)
                        patched['pose_landmarks'] = pc2_data
                        results = patched

            # Serialize (support both local raw-object keys and remote serialized keys)
            pose = self._serialize_landmarks(results, 'pose')
            if not pose:
                pose = self._serialize_landmarks(results, 'pose_landmarks')
            if not pose:
                pose = self._serialize_landmarks(results, 'landmarks')
            if not pose:
                pose = self._serialize_landmarks(results, 'packet_landmarks')
            if not pose:
                compact_pose = _normalize_compact_pose_payload(results)
                if compact_pose:
                    pose = compact_pose

            face = self._serialize_landmarks(results, 'face')
            if not face:
                face = self._serialize_landmarks(results, 'face_landmarks')

            hand = self._serialize_landmarks(results, 'hand')
            if not hand:
                hand = self._serialize_landmarks(results, 'hand_landmarks')
            
            derived = []
            for p_idx, person_pose in enumerate(pose):
                p_metrics = Calculations.get_body_metrics(person_pose)
                if p_idx < len(face):
                     # Need to convert face list back to dict format expected by Calculations
                     # _serialize_landmarks returns list of dicts {'x':...}
                     # Calculations expects list of dicts directly
                    f_metrics = Calculations.get_face_metrics(face[p_idx])
                    p_metrics.update(f_metrics)
                derived.append(p_metrics)
            return pose, face, hand, derived

        # PC1
        p1_pose, p1_face, p1_hand, p1_derived = process_results(pc1_results, is_pc2=False)
        
        # PC2
        p2_pose, p2_face, p2_hand, p2_derived = process_results(pc2_results, is_pc2=True)
        
        # 3D
        p3d_data = []
        joint_confidence = {}
        if pose_3d and 'pose_3d' in pose_3d:
            # Flatten 3D dict to list for storage
            # Format: [{'id': 0, 'x': 1.2, ...}, ...]
            for lm_id, lm_data in pose_3d['pose_3d'].items():
                row = dict(lm_data)
                row['id'] = lm_id
                p3d_data.append(row)
                joint_confidence[int(lm_id)] = {
                    'confidence': float(lm_data.get('visibility', 0.0)),
                    'method': lm_data.get('method', 'unknown'),
                    'reproj_error': lm_data.get('reproj_error')
                }

        combined_derived = {
            'kinematics_3d': pose_3d.get('kinematics_3d', {}) if pose_3d else {},
            'low_reliability_landmarks': pose_3d.get('low_reliability_landmarks', []) if pose_3d else [],
            'joint_confidence': joint_confidence,
            'timestamp_ns': pose_3d.get('timestamp_ns') if pose_3d else None
        }

        kinematics_flat = {}
        confidence_data = {
            'joint_confidence': joint_confidence,
            'low_reliability_landmarks': combined_derived['low_reliability_landmarks']
        }
        if pose_3d and 'kinematics_3d' in pose_3d:
            kinematics_flat = pose_3d['kinematics_3d'].get('flat_export', {})
        
        data = {
            'type': 'multi',
            'timestamp': timestamp,
            # PC1
            'pose': p1_pose, 'face': p1_face, 'hand': p1_hand, 'derived': p1_derived,
            # PC2
            'pc2_pose': p2_pose, 'pc2_face': p2_face, 'pc2_hand': p2_hand, 'pc2_derived': p2_derived,
            # 3D
            'pose_3d': p3d_data,
            'combined_derived': combined_derived,
            'kinematics_flat': kinematics_flat,
            'confidence_data': confidence_data,
        }

        # Final payload trace before enqueue/insert.
        self._save_trace_count += 1
        if self._save_trace_enabled and self._save_trace_count % self._save_trace_every == 0:
            # 3. Calculate people count
            p2_people = len(_extract_pc2_pose_data(pc2_results))
            print(
                f"[SaveTrace][Payload] n={self._save_trace_count} "
                f"p1_pose_people={len(p1_pose)} p2_pose_people={p2_people} "
                f"p3d_pts={len(p3d_data)} kin_keys={len(kinematics_flat)} "
                f"conf_keys={len(confidence_data.get('joint_confidence', {}))}"
            )
        self._enqueue_data(data)

    def _serialize_landmarks(self, results, key_or_attr):
        """Convert MediaPipe results or FrameData results to JSON-serializable list."""
        val = None
        
        # 1. EXTRACT DATA
        if isinstance(results, dict):
             val = results.get(key_or_attr)
             # Fallback: key 'pose' might contain object with 'pose_landmarks'
             if not val and key_or_attr == 'pose_landmarks':
                 val = results.get('pose')
             # Remote compact format fallback: landmarks/packet_landmarks may carry pose.
             if (not val) and key_or_attr in ('pose', 'pose_landmarks', 'landmarks', 'packet_landmarks'):
                 val = (
                     results.get('pose_landmarks')
                     or results.get('landmarks')
                     or results.get('packet_landmarks')
                     or results.get('pose')
                 )
        else:
             val = getattr(results, key_or_attr, None)

        if not val: return []
        
        # 2. NORMALIZE TO LIST OF PEOPLE [(lm1, lm2...), (lm1...)]
        all_people_landmarks = []
        
        # CASE A: MediaPipe Solution Output (Object)
        # It has .pose_landmarks or .face_landmarks attribute which is a LIST of NormalizedLandmarkList
        if hasattr(val, 'pose_landmarks'):
             all_people_landmarks = val.pose_landmarks
        elif hasattr(val, 'face_landmarks'):
             all_people_landmarks = val.face_landmarks
        elif hasattr(val, 'hand_landmarks'):
             all_people_landmarks = val.hand_landmarks
             
        # CASE B: Already a list (Direct access or deserialized)
        elif isinstance(val, list):
             # Detect flat single-person format: [{x,y,z,...}, ...] produced by
             # compact_results_for_sync.  The list-of-lists format has sub-lists,
             # not dicts, as its first element.
             if val and isinstance(val[0], dict) and 'x' in val[0]:
                 all_people_landmarks = [val]  # wrap as single person
             else:
                 all_people_landmarks = val
             
        # CASE C: Single NormalizedLandmarkList (Rare, but possible in some MP versions)
        elif hasattr(val, 'landmark'): # It's a single set of landmarks
             all_people_landmarks = [val]
             
        else:
             # Unknown type or empty
             return []

        if not all_people_landmarks: return []

        # 3. SERIALIZE EACH PERSON
        serialized_people = []
        
        for person in all_people_landmarks:
             # 'person' is a list of landmarks (or NormalizedLandmarkList)
             # OR 'person' is a dict (if already serialized)?
             
             # Check if 'person' is actually a full result object (nested error case)
             if hasattr(person, 'pose_landmarks'): 
                 continue # Skip invalid nesting
                 
             person_data = []
             
             # Iterate landmarks in this person
             try:
                 for lm in person:
                     lm_dict = {}
                     # Handle Obj vs Dict
                     if isinstance(lm, dict):
                         lm_dict = {
                             'x': round(lm.get('x',0), 5), 
                             'y': round(lm.get('y',0), 5), 
                             'z': round(lm.get('z',0), 5)
                         }
                         if 'v' in lm: lm_dict['v'] = lm['v']
                         elif 'conf' in lm: lm_dict['v'] = lm['conf']
                         elif 'visibility' in lm: lm_dict['v'] = lm['visibility']
                     else:
                         # MediaPipe Landmark Object
                         lm_dict = {
                             'x': round(lm.x, 5), 
                             'y': round(lm.y, 5), 
                             'z': round(lm.z, 5)
                         }
                         if hasattr(lm, 'visibility'):
                             lm_dict['v'] = round(lm.visibility, 5)
                     
                     person_data.append(lm_dict)
                 serialized_people.append(person_data)
             except (TypeError, AttributeError):
                 pass # Not iterable / landmark attr access failed

        return serialized_people

    def _worker_loop(self):
        """Background thread to batch insert data."""
        conn = self._get_connection()
        table_name = self.current_table
        
        while self.running or not self.queue.empty():
            try:
                batch = []
                while len(batch) < 50:
                    try:
                        item = self.queue.get(timeout=0.1)
                        batch.append(item)
                    except queue.Empty:
                        break
                
                if not batch:
                    if not self.running: break
                    continue

                c = conn.cursor()
                for item in batch:
                    if item.get('type') == 'multi':
                        # Multi-camera insert (PC1 + PC2 + 3D)
                        cols = "(timestamp, pose_data, face_data, hand_data, derived_data, pc2_pose_data, pc2_face_data, pc2_hand_data, pc2_derived_data, pose_3d_data, combined_derived_data, kinematics_flat_data, confidence_data)"
                        vals = "?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?" if self.db_type == 'sqlite' else "%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s"

                        c.execute(f"INSERT INTO {table_name} {cols} VALUES ({vals})", (
                            item['timestamp'],
                            json.dumps(item['pose']), json.dumps(item['face']), json.dumps(item['hand']), json.dumps(item['derived']),
                            json.dumps(item['pc2_pose']), json.dumps(item['pc2_face']), json.dumps(item['pc2_hand']), json.dumps(item['pc2_derived']),
                            json.dumps(item['pose_3d']), json.dumps(item['combined_derived']),
                            json.dumps(item.get('kinematics_flat', {})), json.dumps(item.get('confidence_data', {}))
                        ))
                    else:
                        # Single camera insert (PC1 only) - Fill others with NULL/Empty
                        cols = "(timestamp, pose_data, face_data, hand_data, derived_data)"
                        vals = "?, ?, ?, ?, ?" if self.db_type == 'sqlite' else "%s, %s, %s, %s, %s"

                        c.execute(f"INSERT INTO {table_name} {cols} VALUES ({vals})", (
                            item['timestamp'],
                            json.dumps(item['pose']), json.dumps(item['face']), json.dumps(item['hand']), json.dumps(item['derived'])
                        ))
                         
                conn.commit()
            except Exception as e:
                print(f"DB Error: {e}")
        conn.close()

    def export_latest_session_csv(self):
        """Export the most recent non-empty session to a CSV string."""
        conn = self._get_connection()
        c = conn.cursor()
        
        # Get most recent session with actual data (skip empty ones)
        c.execute("SELECT id, table_name FROM sessions ORDER BY start_time DESC")
        all_sessions = c.fetchall()
        if not all_sessions:
            conn.close()
            return None

        session_id, table_name = None, None
        for s_id, s_table in all_sessions:
            if not s_table:
                continue
            try:
                cnt = c.execute(f"SELECT COUNT(*) FROM {s_table}").fetchone()[0]
                if cnt > 0:
                    session_id, table_name = s_id, s_table
                    break
            except Exception:
                continue

        if not table_name:
            conn.close()
            return None
             
        # Check columns to decide query type
        # SQLite pragma
        if self.db_type == 'sqlite':
            c.execute(f"PRAGMA table_info({table_name})")
            cols = [info[1] for info in c.fetchall()]
        else:
            # Postgres assumption
            cols = [] # TODO
        
        is_multi = 'pc2_pose_data' in cols

        has_kinematics_flat = 'kinematics_flat_data' in cols

        if is_multi and has_kinematics_flat:
            query = f"SELECT timestamp, pose_data, derived_data, pc2_pose_data, pc2_derived_data, pose_3d_data, kinematics_flat_data FROM {table_name} ORDER BY timestamp ASC"
        elif is_multi:
            query = f"SELECT timestamp, pose_data, derived_data, pc2_pose_data, pc2_derived_data, pose_3d_data FROM {table_name} ORDER BY timestamp ASC"
        else:
            query = f"SELECT timestamp, pose_data, derived_data FROM {table_name} ORDER BY timestamp ASC"
             
        c.execute(query)
        rows = c.fetchall()
        conn.close()
        
        output = io.StringIO()
        writer = csv.writer(output)
        
        # CSV Header
        header = ['Timestamp', 'Source', 'Person', 'Key', 'Value_X', 'Value_Y', 'Value_Z', 'Conf']
        writer.writerow(header)
        
        for row in rows:
            ts = row[0]
            
            # Helper
            def write_data(source, pose_json, derived_json):
                if pose_json:
                    try:
                        people = json.loads(pose_json)
                        for p_i, p in enumerate(people):
                            for l_i, lm in enumerate(p):
                                writer.writerow([ts, source, p_i, l_i, lm.get('x'), lm.get('y'), lm.get('z'), lm.get('v','')])
                    except: pass
                if derived_json:
                    try:
                        derived = json.loads(derived_json)
                        for p_i, m in enumerate(derived):
                            for k, v in m.items():
                                writer.writerow([ts, source, p_i, k, v, '', '', ''])
                    except: pass

            # PC1
            write_data('PC1', row[1], row[2])
            
            if is_multi:
                # PC2
                write_data('PC2', row[3], row[4])
                
                # 3D
                p3d_json = row[5]
                if p3d_json:
                    try:
                        pts = json.loads(p3d_json)
                        # list of dicts {'id':..., 'x':..., ...}
                        for Pt in pts:
                            writer.writerow([ts, '3D', 0, Pt.get('id'), Pt.get('x'), Pt.get('y'), Pt.get('z'), Pt.get('visibility')])
                    except: pass

                # Flat kinematics (optional)
                if has_kinematics_flat and len(row) > 6 and row[6]:
                    try:
                        kin = json.loads(row[6])
                        for k, v in kin.items():
                            writer.writerow([ts, 'KIN', 0, k, v, '', '', ''])
                    except: pass
                      
        return output.getvalue()

    # =========================================================================
    # Dataset Pipeline — JSON Export, Session Archive, Raw Frame Archival
    # =========================================================================

    def export_session_json(self, session_id: str = None) -> Optional[str]:
        """
        Export a session to a structured JSON string.
        
        If session_id is None, exports the latest session.
        Returns JSON string or None.
        """
        conn = self._get_connection()
        c = conn.cursor()

        if session_id:
            placeholder = '%s' if self.db_type == 'postgres' else '?'
            c.execute(f"SELECT id, start_time, table_name FROM sessions WHERE id = {placeholder}", (session_id,))
        else:
            c.execute("SELECT id, start_time, table_name FROM sessions ORDER BY start_time DESC LIMIT 1")
        
        row = c.fetchone()
        if not row:
            conn.close()
            return None
        
        sid, start_time, table_name = row
        if not table_name:
            conn.close()
            return None

        # Determine schema
        if self.db_type == 'sqlite':
            c.execute(f"PRAGMA table_info({table_name})")
            cols = [info[1] for info in c.fetchall()]
        else:
            cols = []

        # Fetch all rows
        c.execute(f"SELECT * FROM {table_name} ORDER BY timestamp ASC")
        rows = c.fetchall()
        conn.close()

        frames = []
        for row_data in rows:
            frame = {'id': row_data[0], 'timestamp': row_data[1]}
            for i, col_name in enumerate(cols):
                if i <= 1:
                    continue  # id and timestamp already added
                val = row_data[i] if i < len(row_data) else None
                if val and isinstance(val, str):
                    try:
                        frame[col_name] = json.loads(val)
                    except (json.JSONDecodeError, TypeError):
                        frame[col_name] = val
                else:
                    frame[col_name] = val
            frames.append(frame)

        export = {
            'session_id': sid,
            'start_time': start_time,
            'table_name': table_name,
            'total_frames': len(frames),
            'schema_columns': cols,
            'frames': frames
        }
        return json.dumps(export, indent=2, default=str)

    def archive_session(self, session_id: str = None, output_dir: str = 'results') -> Optional[str]:
        """
        Archive a session as a ZIP bundle containing:
          - session_metadata.json (session info)
          - data.csv (CSV export)
          - data.json (JSON export)
          - raw_frames/ (if raw frames were saved)
        
        Returns path to ZIP file or None.
        """
        import zipfile
        import os
        import glob

        conn = self._get_connection()
        c = conn.cursor()

        if session_id:
            placeholder = '%s' if self.db_type == 'postgres' else '?'
            c.execute(f"SELECT id, start_time, table_name FROM sessions WHERE id = {placeholder}", (session_id,))
        else:
            c.execute("SELECT id, start_time, table_name FROM sessions ORDER BY start_time DESC LIMIT 1")

        row = c.fetchone()
        conn.close()
        if not row:
            return None

        sid, start_time, table_name = row
        if not table_name:
            return None

        os.makedirs(output_dir, exist_ok=True)
        zip_name = f"{table_name}_archive.zip"
        zip_path = os.path.join(output_dir, zip_name)

        try:
            with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zf:
                # 1. Metadata
                metadata = {
                    'session_id': sid,
                    'start_time': start_time,
                    'table_name': table_name,
                    'archive_created': datetime.now().isoformat()
                }
                zf.writestr('session_metadata.json', json.dumps(metadata, indent=2))

                # 2. CSV export
                csv_data = self.export_latest_session_csv()
                if csv_data:
                    zf.writestr('data.csv', csv_data)

                # 3. JSON export
                json_data = self.export_session_json(sid)
                if json_data:
                    zf.writestr('data.json', json_data)

                # 4. Raw frames (if saved)
                raw_dir = os.path.join(output_dir, 'raw_frames', table_name)
                if os.path.isdir(raw_dir):
                    for fpath in sorted(glob.glob(os.path.join(raw_dir, '*.jpg'))):
                        arcname = os.path.join('raw_frames', os.path.basename(fpath))
                        zf.write(fpath, arcname)

            print(f"[Database] Session archived to {zip_path}")
            return zip_path
        except Exception as e:
            print(f"[Database] Archive error: {e}")
            return None

    def save_raw_frame(self, frame_bgr, frame_number: int, camera_id: str = 'local',
                       output_dir: str = 'results'):
        """
        Save a raw JPEG frame to disk for dataset archival.
        
        Args:
            frame_bgr: BGR numpy array
            frame_number: Sequential frame number
            camera_id: Camera identifier
            output_dir: Base output directory
        """
        import os
        import cv2 as _cv2
        
        if not self.current_table:
            return
        
        raw_dir = os.path.join(output_dir, 'raw_frames', self.current_table)
        os.makedirs(raw_dir, exist_ok=True)
        
        filename = f"{camera_id}_{frame_number:06d}.jpg"
        filepath = os.path.join(raw_dir, filename)
        
        try:
            _cv2.imwrite(filepath, frame_bgr, [_cv2.IMWRITE_JPEG_QUALITY, 95])
        except Exception as e:
            if frame_number % 100 == 0:
                print(f"[Database] Raw frame save error: {e}")

    def get_session_list(self) -> list:
        """Return list of all sessions with metadata."""
        conn = self._get_connection()
        c = conn.cursor()
        c.execute("SELECT id, start_time, table_name FROM sessions ORDER BY start_time DESC")
        rows = c.fetchall()
        conn.close()
        return [
            {'id': r[0], 'start_time': r[1], 'table_name': r[2]}
            for r in rows
        ]
