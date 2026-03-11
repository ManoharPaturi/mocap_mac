"""
Evaluation pipeline for motion capture research reporting.

Outputs (per session):
- per_frame_metrics.csv
- aggregate_metrics.json
- evaluation_table.md
"""

from __future__ import annotations

import csv
import json
import os
import time
import math
import statistics
from collections import deque, defaultdict
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from src.results_report import write_consolidated_results_report


class MotionCaptureEvaluationPipeline:
    """Computes and logs runtime evaluation metrics for each synchronized frame."""

    # MediaPipe pose edges (subset) for stable anthropometric tracking.
    _BONE_PAIRS: List[Tuple[int, int]] = [
        (11, 12),  # shoulders
        (23, 24),  # hips
        (11, 23),  # left torso
        (12, 24),  # right torso
        (13, 15),  # left upper/lower arm
        (14, 16),  # right upper/lower arm
        (23, 25),  # left thigh
        (24, 26),  # right thigh
        (25, 27),  # left shank
        (26, 28),  # right shank
    ]

    _DEPTH_JOINTS: List[int] = [11, 12, 23, 24]  # torso anchor for depth stability

    _FRAME_FIELDS: List[str] = [
        "frame_index",
        "timestamp_ns",
        "reconstruction_accuracy",
        "mpjpe_m",
        "reprojection_error_px",
        "epipolar_error_px",
        "bone_length_variance_m",
        "joint_jitter_m",
        "occlusion_recovery_ms",
        "calibration_rms_px",
        "pipeline_latency_ms",
        "fps",
        "network_latency_ms",
        "packet_loss_rate",
        "synchronization_error_ms",
        "triangulation_success_rate",
        "depth_stability_m",
        "cpu_percent",
        "memory_mb",
        "triangulated_points",
        "total_points",
    ]

    def __init__(self, output_root: str = "results", flush_every: int = 60):
        self.output_root = output_root
        self.flush_every = max(1, int(flush_every))

        self._db = None
        self._session_label = None
        self._session_dir = None
        self._csv_file = None
        self._csv_writer = None

        self._frame_counter = 0
        self._rows: List[Dict[str, Any]] = []
        self._bone_hist: Dict[Tuple[int, int], deque] = {
            pair: deque(maxlen=120) for pair in self._BONE_PAIRS
        }
        self._depth_hist: deque = deque(maxlen=120)
        self._prev_pose: Optional[Dict[int, Dict[str, Any]]] = None
        self._prev_ts_ns: Optional[int] = None
        self._ground_truth_by_frame: Dict[int, Dict[int, Dict[str, float]]] = {}
        self._gt_loaded_from: Optional[str] = None
        self._calibration_rms_px: Optional[float] = None
        self._fundamental_matrix: Optional[np.ndarray] = None
        self._camera_image_sizes: Dict[str, Tuple[float, float]] = {}
        self._occlusion_start_ns: Dict[int, int] = {}
        self._occlusion_recovery_history_ms: Dict[int, deque] = defaultdict(lambda: deque(maxlen=120))
        self._prev_fps_ts_ns: Optional[int] = None

        # Lazy import psutil when available.
        self._proc = None
        try:
            import psutil  # type: ignore
            self._proc = psutil.Process(os.getpid())
            self._proc.cpu_percent(interval=None)
        except Exception:
            self._proc = None

    def bind_database(self, db: Any):
        self._db = db

    def set_ground_truth(self, dataset_path: Optional[str]):
        """Load ground-truth joints for MPJPE.

        Supported JSON schema:
        {
          "frames": [
            {"frame_index": 1, "joints": {"11": {"x":...,"y":...,"z":...}, ...}},
            ...
          ]
        }
        """
        if not dataset_path:
            return
        if not os.path.exists(dataset_path):
            return
        try:
            with open(dataset_path, "r", encoding="utf-8") as f:
                payload = json.load(f)
        except Exception:
            return

        frames = payload.get("frames") if isinstance(payload, dict) else None
        if not isinstance(frames, list):
            return

        parsed: Dict[int, Dict[int, Dict[str, float]]] = {}
        for frame in frames:
            if not isinstance(frame, dict):
                continue
            try:
                frame_idx = int(frame.get("frame_index"))
            except Exception:
                continue
            joints = frame.get("joints")
            if not isinstance(joints, dict):
                continue
            norm_joints: Dict[int, Dict[str, float]] = {}
            for jid, xyz in joints.items():
                if not isinstance(xyz, dict):
                    continue
                try:
                    jid_i = int(jid)
                    norm_joints[jid_i] = {
                        "x": float(xyz.get("x", 0.0)),
                        "y": float(xyz.get("y", 0.0)),
                        "z": float(xyz.get("z", 0.0)),
                    }
                except Exception:
                    continue
            if norm_joints:
                parsed[frame_idx] = norm_joints

        if parsed:
            self._ground_truth_by_frame = parsed
            self._gt_loaded_from = dataset_path

    def set_calibration(self, calibration: Any):
        """Bind calibration metadata and precompute a fundamental matrix for epipolar error."""
        self._calibration_rms_px = None
        self._fundamental_matrix = None
        self._camera_image_sizes = {}
        if calibration is None:
            return

        try:
            metadata = getattr(calibration, "metadata", {}) or {}
            rms = metadata.get("rms_error")
            if rms is not None:
                self._calibration_rms_px = float(rms)
        except Exception:
            self._calibration_rms_px = None

        try:
            cams = getattr(calibration, "cameras", {}) or {}
            c1 = cams.get("local_cam")
            c2 = cams.get("cam_0")
            if c1 is None or c2 is None:
                return
            K1 = np.array(c1.intrinsic_matrix, dtype=float)
            K2 = np.array(c2.intrinsic_matrix, dtype=float)
            R1 = np.array(c1.rotation, dtype=float)
            t1 = np.array(c1.translation, dtype=float).reshape(3, 1)
            R2 = np.array(c2.rotation, dtype=float)
            t2 = np.array(c2.translation, dtype=float).reshape(3, 1)

            # Relative transform from cam1 to cam2
            R = R2 @ R1.T
            t = t2 - (R @ t1)

            tx = np.array([
                [0.0, -t[2, 0], t[1, 0]],
                [t[2, 0], 0.0, -t[0, 0]],
                [-t[1, 0], t[0, 0], 0.0],
            ], dtype=float)
            E = tx @ R
            self._fundamental_matrix = np.linalg.inv(K2).T @ E @ np.linalg.inv(K1)
            for cam_id, cal in cams.items():
                img_size = getattr(cal, "image_size", None)
                if isinstance(img_size, (tuple, list)) and len(img_size) == 2:
                    self._camera_image_sizes[cam_id] = (float(img_size[0]), float(img_size[1]))
        except Exception:
            self._fundamental_matrix = None

    def record_frame(
        self,
        synced_frames: List[Any],
        pose_3d: Optional[Dict[str, Any]],
        latency_stats: Dict[str, Dict[str, float]],
        frames_received: Dict[str, int],
        sequence_gaps: Dict[str, int],
    ):
        self._ensure_outputs()
        self._frame_counter += 1

        pose_points = {}
        if isinstance(pose_3d, dict):
            pose_points = pose_3d.get("pose_3d", {}) or {}

        triangulated_points = 0
        total_points = len(pose_points)
        reproj_vals: List[float] = []
        for _, pt in pose_points.items():
            if not isinstance(pt, dict):
                continue
            if pt.get("method") == "triangulated":
                triangulated_points += 1
            if pt.get("reproj_error") is not None:
                try:
                    reproj_vals.append(float(pt.get("reproj_error")))
                except Exception:
                    pass

        reproj_error_px = self._safe_mean(reproj_vals)
        mpjpe_m = self._safe_metric(self._compute_mpjpe, pose_points, self._frame_counter)
        triangulation_success_rate = (
            float(triangulated_points) / float(total_points) if total_points > 0 else 0.0
        )
        epipolar_error_px = self._safe_metric(self._compute_epipolar_error_px, synced_frames)

        bone_length_variance_m = self._safe_metric(self._compute_bone_length_variance, pose_points)
        joint_jitter_m = self._safe_metric(self._compute_joint_jitter, pose_points, pose_3d)
        occlusion_recovery_ms = self._safe_metric(self._compute_occlusion_recovery_ms, pose_points, pose_3d)
        depth_stability_m = self._safe_metric(self._compute_depth_stability, pose_points)

        # Reconstruction accuracy proxy in [0, 1].
        reproj_term = 1.0 / (1.0 + max(0.0, reproj_error_px))
        reconstruction_accuracy = max(0.0, min(1.0, reproj_term * triangulation_success_rate))

        network_latency_ms = self._safe_metric(self._compute_network_latency_ms, synced_frames)
        synchronization_error_ms = self._safe_metric(self._compute_sync_error_ms, synced_frames)
        packet_loss_rate = self._safe_metric(self._compute_packet_loss_rate, frames_received, sequence_gaps)

        timestamp_ns = 0
        if synced_frames:
            try:
                timestamp_ns = int(sum(int(f.timestamp) for f in synced_frames) / len(synced_frames))
            except Exception:
                timestamp_ns = int(time.time() * 1e9)

        pipeline_latency_ms = self._safe_metric(self._extract_stage, latency_stats, "total")
        fps = self._safe_metric(self._estimate_fps, latency_stats, pipeline_latency_ms, timestamp_ns)

        cpu_percent, memory_mb = self._resource_usage()

        row = {
            "frame_index": self._frame_counter,
            "timestamp_ns": timestamp_ns,
            "reconstruction_accuracy": reconstruction_accuracy,
            "mpjpe_m": mpjpe_m,
            "reprojection_error_px": reproj_error_px,
            "epipolar_error_px": epipolar_error_px,
            "bone_length_variance_m": bone_length_variance_m,
            "joint_jitter_m": joint_jitter_m,
            "occlusion_recovery_ms": occlusion_recovery_ms,
            "calibration_rms_px": float(self._calibration_rms_px or 0.0),
            "pipeline_latency_ms": pipeline_latency_ms,
            "fps": fps,
            "network_latency_ms": network_latency_ms,
            "packet_loss_rate": packet_loss_rate,
            "synchronization_error_ms": synchronization_error_ms,
            "triangulation_success_rate": triangulation_success_rate,
            "depth_stability_m": depth_stability_m,
            "cpu_percent": cpu_percent,
            "memory_mb": memory_mb,
            "triangulated_points": triangulated_points,
            "total_points": total_points,
        }

        self._rows.append(row)
        self._append_csv_row(row)

        if (self._frame_counter % self.flush_every) == 0:
            self._write_aggregate_reports()

    @staticmethod
    def _safe_metric(fn, *args):
        try:
            return fn(*args)
        except Exception:
            return 0.0

    def finalize(self):
        self._write_aggregate_reports()
        if self._csv_file is not None:
            try:
                self._csv_file.close()
            except Exception:
                pass
            self._csv_file = None
            self._csv_writer = None

    # ---------------------------
    # Metric computations
    # ---------------------------

    def _compute_bone_length_variance(self, pose_points: Dict[Any, Dict[str, Any]]) -> float:
        frame_sq_errors: List[float] = []
        for pair in self._BONE_PAIRS:
            a = pose_points.get(pair[0])
            b = pose_points.get(pair[1])
            if not isinstance(a, dict) or not isinstance(b, dict):
                continue
            try:
                d = math.dist(
                    [float(a.get("x", 0.0)), float(a.get("y", 0.0)), float(a.get("z", 0.0))],
                    [float(b.get("x", 0.0)), float(b.get("y", 0.0)), float(b.get("z", 0.0))],
                )
            except Exception:
                continue
            hist = self._bone_hist[pair]
            hist.append(d)
            if len(hist) >= 5:
                mu = sum(hist) / len(hist)
                frame_sq_errors.append((d - mu) ** 2)
        if not frame_sq_errors:
            return 0.0
        return math.sqrt(sum(frame_sq_errors) / len(frame_sq_errors))

    def _compute_mpjpe(self, pose_points: Dict[Any, Dict[str, Any]], frame_index: int) -> float:
        """Mean Per Joint Position Error vs loaded ground truth (meters)."""
        gt = self._ground_truth_by_frame.get(frame_index)
        if not gt:
            return 0.0
        errs: List[float] = []
        for jid, gt_pt in gt.items():
            pred = pose_points.get(jid)
            if not isinstance(pred, dict):
                continue
            try:
                e = math.dist(
                    [float(pred.get("x", 0.0)), float(pred.get("y", 0.0)), float(pred.get("z", 0.0))],
                    [float(gt_pt.get("x", 0.0)), float(gt_pt.get("y", 0.0)), float(gt_pt.get("z", 0.0))],
                )
                errs.append(e)
            except Exception:
                continue
        return self._safe_mean(errs)

    def _compute_epipolar_error_px(self, synced_frames: List[Any]) -> float:
        """Mean point-to-epipolar-line distance across matched joints (pixels)."""
        F = self._fundamental_matrix
        if F is None or len(synced_frames or []) < 2:
            return 0.0

        local = next((f for f in synced_frames if getattr(f, "camera_id", "") == "local_cam"), None)
        remote = next((f for f in synced_frames if getattr(f, "camera_id", "") == "cam_0"), None)
        if local is None or remote is None:
            return 0.0

        p1 = self._extract_2d_points(local)
        p2 = self._extract_2d_points(remote)
        if not p1 or not p2:
            return 0.0

        errs: List[float] = []
        joint_ids = set(p1.keys()) & set(p2.keys())
        for jid in joint_ids:
            x1, y1 = p1[jid]
            x2, y2 = p2[jid]
            l2 = F @ np.array([x1, y1, 1.0], dtype=float)
            a, b, c = float(l2[0]), float(l2[1]), float(l2[2])
            denom = math.sqrt(a * a + b * b)
            if denom <= 1e-9:
                continue
            d = abs(a * x2 + b * y2 + c) / denom
            errs.append(d)

        return self._safe_mean(errs)

    def _compute_joint_jitter(self, pose_points: Dict[Any, Dict[str, Any]], pose_3d: Optional[Dict[str, Any]]) -> float:
        ts_ns = 0
        if isinstance(pose_3d, dict):
            try:
                ts_ns = int(pose_3d.get("timestamp_ns") or 0)
            except Exception:
                ts_ns = 0

        if ts_ns <= 0:
            ts_ns = int(time.time() * 1e9)

        if self._prev_pose is None or self._prev_ts_ns is None or ts_ns <= self._prev_ts_ns:
            self._prev_pose = pose_points
            self._prev_ts_ns = ts_ns
            return 0.0

        dt = max(1e-6, (ts_ns - self._prev_ts_ns) / 1e9)
        deltas = []
        for jid, pt in pose_points.items():
            prev = self._prev_pose.get(jid) if isinstance(self._prev_pose, dict) else None
            if not isinstance(pt, dict) or not isinstance(prev, dict):
                continue
            try:
                d = math.dist(
                    [float(pt.get("x", 0.0)), float(pt.get("y", 0.0)), float(pt.get("z", 0.0))],
                    [float(prev.get("x", 0.0)), float(prev.get("y", 0.0)), float(prev.get("z", 0.0))],
                )
                deltas.append(d / dt)
            except Exception:
                continue

        self._prev_pose = pose_points
        self._prev_ts_ns = ts_ns
        if not deltas:
            return 0.0

        # Jitter proxy: speed dispersion converted back to displacement scale.
        speed_std = statistics.pstdev(deltas) if len(deltas) > 1 else deltas[0]
        return speed_std * dt

    def _compute_depth_stability(self, pose_points: Dict[Any, Dict[str, Any]]) -> float:
        z_vals = []
        for jid in self._DEPTH_JOINTS:
            pt = pose_points.get(jid)
            if isinstance(pt, dict):
                try:
                    z_vals.append(float(pt.get("z", 0.0)))
                except Exception:
                    pass
        if z_vals:
            self._depth_hist.append(sum(z_vals) / len(z_vals))
        if len(self._depth_hist) < 5:
            return 0.0
        return statistics.pstdev(self._depth_hist)

    def _compute_occlusion_recovery_ms(self, pose_points: Dict[Any, Dict[str, Any]], pose_3d: Optional[Dict[str, Any]]) -> float:
        """Track recovery time from low-confidence to recovered confidence per joint."""
        ts_ns = 0
        if isinstance(pose_3d, dict):
            try:
                ts_ns = int(pose_3d.get("timestamp_ns") or 0)
            except Exception:
                ts_ns = 0
        if ts_ns <= 0:
            ts_ns = time.time_ns()

        low_thresh = 0.35
        recover_thresh = 0.6

        for jid, pt in pose_points.items():
            if not isinstance(pt, dict):
                continue
            try:
                conf = float(pt.get("visibility", 0.0))
            except Exception:
                conf = 0.0

            if conf < low_thresh:
                if jid not in self._occlusion_start_ns:
                    self._occlusion_start_ns[jid] = ts_ns
            elif conf >= recover_thresh and jid in self._occlusion_start_ns:
                delta_ms = (ts_ns - self._occlusion_start_ns[jid]) / 1e6
                if delta_ms >= 0:
                    self._occlusion_recovery_history_ms[jid].append(float(delta_ms))
                del self._occlusion_start_ns[jid]

        all_vals: List[float] = []
        for vals in self._occlusion_recovery_history_ms.values():
            all_vals.extend(list(vals))
        return self._safe_mean(all_vals)

    def _compute_network_latency_ms(self, synced_frames: List[Any]) -> float:
        vals = []
        now_ns = time.time_ns()
        for f in synced_frames or []:
            try:
                sent_ns = int(getattr(f, "timestamp", 0) or 0)
                recv_at = float(getattr(f, "received_at", 0.0) or 0.0)
                recv_ns = int(recv_at * 1e9) if recv_at > 0 else now_ns
                if sent_ns > 0:
                    vals.append(max(0.0, (recv_ns - sent_ns) / 1e6))
            except Exception:
                continue
        return self._safe_mean(vals)

    def _compute_sync_error_ms(self, synced_frames: List[Any]) -> float:
        if not synced_frames:
            return 0.0
        try:
            timestamps = [int(getattr(f, "timestamp", 0) or 0) for f in synced_frames]
            timestamps = [t for t in timestamps if t > 0]
            if len(timestamps) < 2:
                return 0.0
            return (max(timestamps) - min(timestamps)) / 1e6
        except Exception:
            return 0.0

    def _compute_packet_loss_rate(self, frames_received: Dict[str, int], sequence_gaps: Dict[str, int]) -> float:
        recv_total = int(sum(int(v) for v in (frames_received or {}).values()))
        gap_total = int(sum(int(v) for v in (sequence_gaps or {}).values()))
        denom = recv_total + gap_total
        if denom <= 0:
            return 0.0
        return float(gap_total) / float(denom)

    def _extract_stage(self, latency_stats: Dict[str, Dict[str, float]], stage: str) -> float:
        if not isinstance(latency_stats, dict):
            return 0.0
        stage_info = latency_stats.get(stage) or {}
        try:
            return float(stage_info.get("mean_ms", 0.0) or 0.0)
        except Exception:
            return 0.0

    def _estimate_fps(
        self,
        latency_stats: Dict[str, Dict[str, float]],
        pipeline_latency_ms: float,
        timestamp_ns: int,
    ) -> float:
        # Prefer timestamp deltas from synchronized frame clocks when available.
        if isinstance(timestamp_ns, int) and timestamp_ns > 0:
            if self._prev_fps_ts_ns is not None and timestamp_ns > self._prev_fps_ts_ns:
                dt_s = (timestamp_ns - self._prev_fps_ts_ns) / 1e9
                if dt_s > 1e-6:
                    inst_fps = 1.0 / dt_s
                    if 0.5 <= inst_fps <= 240.0:
                        self._prev_fps_ts_ns = timestamp_ns
                        return inst_fps
            self._prev_fps_ts_ns = timestamp_ns

        # Fallback to latency-derived estimates, bounded to realistic runtime ranges.
        cap_ms = self._extract_stage(latency_stats, "capture")
        if cap_ms > 0:
            cap_fps = 1000.0 / cap_ms
            if 0.5 <= cap_fps <= 240.0:
                return cap_fps

        if pipeline_latency_ms > 0:
            pipe_fps = 1000.0 / pipeline_latency_ms
            if 0.5 <= pipe_fps <= 240.0:
                return pipe_fps

        return 0.0

    def _resource_usage(self) -> Tuple[float, float]:
        if self._proc is None:
            return 0.0, 0.0
        try:
            cpu = float(self._proc.cpu_percent(interval=None))
            mem = float(self._proc.memory_info().rss) / (1024.0 * 1024.0)
            return cpu, mem
        except Exception:
            return 0.0, 0.0

    def _extract_2d_points(self, frame: Any) -> Dict[int, Tuple[float, float]]:
        """Extract normalized 2D pose points from compact payload formats."""
        points: Dict[int, Tuple[float, float]] = {}
        try:
            results = getattr(frame, "results", {}) or {}
        except Exception:
            results = {}

        if not isinstance(results, dict):
            return points

        raw = results.get("packet_landmarks") or results.get("landmarks") or results.get("pose_landmarks") or []
        if isinstance(raw, list) and raw and isinstance(raw[0], list):
            raw = raw[0]
        if not (isinstance(raw, list) and raw and isinstance(raw[0], dict)):
            return points

        cam_id = str(getattr(frame, "camera_id", ""))
        w, h = self._camera_image_sizes.get(cam_id, (1.0, 1.0))

        for idx, lm in enumerate(raw):
            try:
                x = float(lm.get("x", 0.0))
                y = float(lm.get("y", 0.0))
                # Compact payload is normalized to [0,1] on send path.
                points[idx] = (x * w, y * h)
            except Exception:
                continue
        return points

    # ---------------------------
    # Reporting / files
    # ---------------------------

    def _ensure_outputs(self):
        session_label = self._resolve_session_label()
        if self._session_label == session_label and self._csv_writer is not None:
            return

        self._session_label = session_label
        self._session_dir = os.path.join(self.output_root, session_label, "evaluation")
        os.makedirs(self._session_dir, exist_ok=True)

        if self._csv_file is not None:
            try:
                self._csv_file.close()
            except Exception:
                pass

        csv_path = os.path.join(self._session_dir, "per_frame_metrics.csv")
        file_exists = os.path.exists(csv_path)
        self._csv_file = open(csv_path, "a", newline="", encoding="utf-8")
        self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=self._FRAME_FIELDS)
        if not file_exists or os.path.getsize(csv_path) == 0:
            self._csv_writer.writeheader()
            self._csv_file.flush()

    def _resolve_session_label(self) -> str:
        # Keep a stable `session_*` label once recording has started so post-stop
        # live processing does not overwrite latest recorded artifact pointers.
        if isinstance(self._session_label, str) and self._session_label.startswith("session_"):
            return self._session_label

        if self._db is not None:
            table_name = getattr(self._db, "current_table", None)
            if isinstance(table_name, str) and table_name.strip():
                return table_name.strip()

        if isinstance(self._session_label, str) and self._session_label.strip():
            return self._session_label.strip()

        return "live_" + time.strftime("%Y%m%d_%H%M%S")

    def _append_csv_row(self, row: Dict[str, Any]):
        if self._csv_writer is None or self._csv_file is None:
            return
        out = {k: row.get(k) for k in self._FRAME_FIELDS}
        self._csv_writer.writerow(out)
        self._csv_file.flush()

    def _write_aggregate_reports(self):
        if not self._rows:
            return
        self._ensure_outputs()

        numeric_keys = [
            k for k in self._FRAME_FIELDS
            if k not in ("frame_index", "timestamp_ns")
        ]
        summary = {
            "session_label": self._session_label,
            "frames_evaluated": len(self._rows),
            "metrics": {},
            "ground_truth_dataset": self._gt_loaded_from,
            "calibration_rms_px": float(self._calibration_rms_px or 0.0),
            "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        }

        for key in numeric_keys:
            vals = []
            for r in self._rows:
                try:
                    vals.append(float(r.get(key, 0.0)))
                except Exception:
                    pass
            summary["metrics"][key] = self._describe(vals)

        json_path = os.path.join(self._session_dir, "aggregate_metrics.json")
        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2)

        md_path = os.path.join(self._session_dir, "evaluation_table.md")
        with open(md_path, "w", encoding="utf-8") as f:
            f.write(self._render_markdown_table(summary))

        self._generate_plots()
        self._write_latest_artifacts_index(summary, json_path, md_path)
        self._refresh_consolidated_report()

    def _refresh_consolidated_report(self):
        try:
            repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
            write_consolidated_results_report(repo_root=repo_root, output_root=self.output_root)
        except Exception:
            # Reporting should never interrupt live capture/evaluation.
            pass

    def _write_latest_artifacts_index(self, summary: Dict[str, Any], json_path: str, md_path: str):
        """Write a stable pointer file so tooling can always find newest evaluation outputs."""
        if not self._session_dir:
            return

        payload = {
            "session_label": summary.get("session_label"),
            "frames_evaluated": summary.get("frames_evaluated"),
            "generated_at": summary.get("generated_at"),
            "ground_truth_dataset": summary.get("ground_truth_dataset"),
            "calibration_rms_px": summary.get("calibration_rms_px"),
            "artifacts": {
                "per_frame_csv": os.path.join(self._session_dir, "per_frame_metrics.csv"),
                "aggregate_json": json_path,
                "evaluation_table_md": md_path,
                "latency_kde_png": os.path.join(self._session_dir, "latency_kde.png"),
                "bone_variance_line_png": os.path.join(self._session_dir, "bone_variance_line.png"),
                "jitter_scatter_png": os.path.join(self._session_dir, "jitter_scatter.png"),
            }
        }

        stable_path = os.path.join(self.output_root, "latest_evaluation_artifacts.json")
        try:
            with open(stable_path, "w", encoding="utf-8") as f:
                json.dump(payload, f, indent=2)
        except Exception:
            pass

    def _generate_plots(self):
        """Create publication-ready figures using matplotlib/seaborn."""
        if not self._rows or not self._session_dir:
            return
        try:
            import matplotlib.pyplot as plt
            import seaborn as sns
            import pandas as pd
        except Exception:
            return

        try:
            sns.set_theme(style="whitegrid", context="talk")
            df = pd.DataFrame(self._rows)

            # 1) Latency KDE
            plt.figure(figsize=(8, 5))
            sns.kdeplot(data=df, x="pipeline_latency_ms", fill=True, color="#2E86AB")
            plt.title("Pipeline Latency Distribution")
            plt.xlabel("Latency (ms)")
            plt.ylabel("Density")
            plt.tight_layout()
            plt.savefig(os.path.join(self._session_dir, "latency_kde.png"), dpi=220)
            plt.close()

            # 2) Bone variance line chart
            plt.figure(figsize=(10, 5))
            sns.lineplot(data=df, x="frame_index", y="bone_length_variance_m", color="#F18F01")
            plt.title("Bone Length Variance Over Time")
            plt.xlabel("Frame")
            plt.ylabel("Variance (m)")
            plt.tight_layout()
            plt.savefig(os.path.join(self._session_dir, "bone_variance_line.png"), dpi=220)
            plt.close()

            # 3) Jitter scatter plot
            plt.figure(figsize=(9, 5))
            sns.scatterplot(data=df, x="frame_index", y="joint_jitter_m", s=18, alpha=0.7, color="#C73E1D")
            plt.title("Joint Jitter Scatter")
            plt.xlabel("Frame")
            plt.ylabel("Jitter (m)")
            plt.tight_layout()
            plt.savefig(os.path.join(self._session_dir, "jitter_scatter.png"), dpi=220)
            plt.close()
        except Exception:
            pass

    def _describe(self, vals: List[float]) -> Dict[str, float]:
        if not vals:
            return {
                "mean": 0.0,
                "std": 0.0,
                "min": 0.0,
                "p95": 0.0,
                "max": 0.0,
            }
        svals = sorted(vals)
        n = len(svals)
        p95 = svals[min(n - 1, int(math.ceil(0.95 * n)) - 1)]
        return {
            "mean": float(sum(svals) / n),
            "std": float(statistics.pstdev(svals) if n > 1 else 0.0),
            "min": float(svals[0]),
            "p95": float(p95),
            "max": float(svals[-1]),
        }

    def _render_markdown_table(self, summary: Dict[str, Any]) -> str:
        lines = []
        lines.append("# Motion Capture Evaluation Summary")
        lines.append("")
        lines.append(f"- Session: `{summary.get('session_label', 'unknown')}`")
        lines.append(f"- Frames evaluated: `{summary.get('frames_evaluated', 0)}`")
        lines.append(f"- Generated at: `{summary.get('generated_at', '')}`")
        lines.append(f"- Calibration RMS (px): `{float(summary.get('calibration_rms_px', 0.0)):.4f}`")
        if summary.get("ground_truth_dataset"):
            lines.append(f"- Ground truth source: `{summary.get('ground_truth_dataset')}`")
        lines.append("")
        lines.append("| Metric | Mean | Std | P95 | Min | Max |")
        lines.append("|---|---:|---:|---:|---:|---:|")
        metrics = summary.get("metrics", {})
        for metric_name in sorted(metrics.keys()):
            m = metrics.get(metric_name, {})
            lines.append(
                "| {name} | {mean:.4f} | {std:.4f} | {p95:.4f} | {minv:.4f} | {maxv:.4f} |".format(
                    name=metric_name,
                    mean=float(m.get("mean", 0.0)),
                    std=float(m.get("std", 0.0)),
                    p95=float(m.get("p95", 0.0)),
                    minv=float(m.get("min", 0.0)),
                    maxv=float(m.get("max", 0.0)),
                )
            )
        lines.append("")
        return "\n".join(lines)

    @staticmethod
    def _safe_mean(values: List[float]) -> float:
        if not values:
            return 0.0
        return float(sum(values) / len(values))
