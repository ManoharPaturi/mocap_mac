#!/usr/bin/env python3
"""
Offline validation for the 4-layer self-aware mocap database.

Flow:
1) Load Layer 0 timeline (frames)
2) Load Layer 2 joints_3d per frame
3) Recompute angles offline
4) Compare with Layer 3 kinematics_3d
5) Store Layer 4 validation summary + per-angle comparisons
"""

import argparse
import json
import os
import sys
import time
from collections import defaultdict
from datetime import datetime
from typing import Any, Dict, List


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from src.database import MocapDB  # noqa: E402
from src.kinematics_engine import KinematicsEngine  # noqa: E402


def _insert_comparisons(db: MocapDB, rows: List[Dict[str, Any]]) -> None:
    if not rows:
        return

    conn = db._get_connection()
    cur = conn.cursor()

    if db.db_type == 'postgres':
        sql = (
            "INSERT INTO validation_angle_comparison "
            "(validation_run_id, session_id, frame_id, angle_name, realtime_angle, offline_angle, error_deg, pass_flag) "
            "VALUES (%s, %s, %s, %s, %s, %s, %s, %s)"
        )
    else:
        sql = (
            "INSERT INTO validation_angle_comparison "
            "(validation_run_id, session_id, frame_id, angle_name, realtime_angle, offline_angle, error_deg, pass_flag) "
            "VALUES (?, ?, ?, ?, ?, ?, ?, ?)"
        )

    payload = [
        (
            row['validation_run_id'],
            row['session_id'],
            row['frame_id'],
            row['angle_name'],
            row['realtime_angle'],
            row['offline_angle'],
            row['error_deg'],
            row['pass_flag'],
        )
        for row in rows
    ]

    cur.executemany(sql, payload)
    conn.commit()
    conn.close()


def _record_artifact(db: MocapDB, validation_run_id: int, session_id: str, artifact_type: str, path: str, metadata: Dict[str, Any]) -> None:
    conn = db._get_connection()
    cur = conn.cursor()

    if db.db_type == 'postgres':
        sql = (
            "INSERT INTO validation_artifacts "
            "(validation_run_id, session_id, artifact_type, file_path, metadata_json, created_at) "
            "VALUES (%s, %s, %s, %s, %s, %s)"
        )
    else:
        sql = (
            "INSERT INTO validation_artifacts "
            "(validation_run_id, session_id, artifact_type, file_path, metadata_json, created_at) "
            "VALUES (?, ?, ?, ?, ?, ?)"
        )

    cur.execute(
        sql,
        (
            validation_run_id,
            session_id,
            artifact_type,
            path,
            json.dumps(metadata),
            datetime.now().isoformat(),
        ),
    )
    conn.commit()
    conn.close()


def validate_session(session_id: str, threshold: float = 2.0, max_frames: int = 0) -> Dict[str, Any]:
    db = MocapDB()
    frames = db.get_session_frames(session_id)
    if max_frames and max_frames > 0:
        frames = frames[:max_frames]

    if not frames:
        raise RuntimeError(f"No frames found for session: {session_id}")

    engine = KinematicsEngine()

    comparisons: List[Dict[str, Any]] = []
    errors: List[float] = []
    per_angle_errors = defaultdict(list)

    for frame_id, timestamp_ms, _frame_index in frames:
        joints_3d = db.get_joints_3d_for_frame(frame_id)
        if not joints_3d:
            continue

        realtime = db.get_kinematics_3d_for_frame(frame_id)
        if not realtime:
            continue

        offline = engine.process_frame(
            joints_3d=joints_3d,
            timestamp=int(float(timestamp_ms) * 1_000_000),
            compute_derivatives=False,
            min_confidence=0.0,
        )

        offline_angles = {
            name: item.angle for name, item in offline.get('angles', {}).items()
        }

        common_names = sorted(set(realtime.keys()) & set(offline_angles.keys()))
        for name in common_names:
            realtime_val = float(realtime[name]['angle'])
            offline_val = float(offline_angles[name])
            error = abs(offline_val - realtime_val)
            ok = 1 if error <= threshold else 0

            errors.append(error)
            per_angle_errors[name].append(error)
            comparisons.append(
                {
                    'frame_id': frame_id,
                    'angle_name': name,
                    'realtime_angle': realtime_val,
                    'offline_angle': offline_val,
                    'error_deg': error,
                    'pass_flag': ok,
                }
            )

    if not comparisons:
        raise RuntimeError(
            f"No comparable angle rows for session {session_id}. "
            "Ensure kinematics_3d and joints_3d are populated for overlapping frames."
        )

    passed = sum(row['pass_flag'] for row in comparisons)
    total = len(comparisons)
    pass_rate = (passed / total) * 100.0
    mean_error = sum(errors) / total
    max_error = max(errors)

    validation_run_id = db.save_validation_run(
        num_frames=len(frames),
        num_angles=total,
        pass_rate=pass_rate,
        mean_error=mean_error,
        max_error=max_error,
        threshold=threshold,
        session_id=session_id,
    )

    for row in comparisons:
        row['validation_run_id'] = validation_run_id
        row['session_id'] = session_id

    _insert_comparisons(db, comparisons)

    output_dir = os.path.join(PROJECT_ROOT, 'results', 'validation')
    os.makedirs(output_dir, exist_ok=True)
    output_path = os.path.join(
        output_dir,
        f"validation_report_{session_id}_{int(time.time())}.json",
    )

    angle_stats = {
        angle_name: {
            'count': len(vals),
            'mean_error': sum(vals) / len(vals),
            'max_error': max(vals),
        }
        for angle_name, vals in per_angle_errors.items()
        if vals
    }

    report = {
        'session_id': session_id,
        'validation_run_id': validation_run_id,
        'created_at': datetime.now().isoformat(),
        'frames_checked': len(frames),
        'angles_compared': total,
        'pass_rate': pass_rate,
        'mean_error': mean_error,
        'max_error': max_error,
        'threshold': threshold,
        'per_angle_stats': angle_stats,
        'top_failures': sorted(
            (r for r in comparisons if not r['pass_flag']),
            key=lambda item: item['error_deg'],
            reverse=True,
        )[:20],
    }

    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(report, f, indent=2)

    if validation_run_id is not None:
        _record_artifact(
            db,
            validation_run_id,
            session_id,
            artifact_type='validation_report_json',
            path=output_path,
            metadata={'angles_compared': total, 'threshold': threshold},
        )

    return report


def _print_report(report: Dict[str, Any]) -> None:
    print('=' * 60)
    print('  VALIDATION REPORT')
    print('=' * 60)
    print(f"Session ID:            {report['session_id']}")
    print(f"Validation Run ID:     {report['validation_run_id']}")
    print(f"Frames Checked:        {report['frames_checked']}")
    print(f"Angles Compared:       {report['angles_compared']}")
    print(f"Pass Rate:             {report['pass_rate']:.2f}%")
    print(f"Mean Error:            {report['mean_error']:.3f}°")
    print(f"Max Error:             {report['max_error']:.3f}°")
    print(f"Error Threshold:       {report['threshold']:.3f}°")
    print('=' * 60)

    if report['pass_rate'] >= 95.0:
        print('✅ Validation passed')
    else:
        print('⚠️ Validation found significant drift; inspect top_failures in report JSON')


def main() -> None:
    parser = argparse.ArgumentParser(description='Validate stored kinematics against offline recomputation.')
    parser.add_argument('--session', required=True, help='Session ID to validate')
    parser.add_argument('--threshold', type=float, default=2.0, help='Error threshold in degrees (default: 2.0)')
    parser.add_argument('--max-frames', type=int, default=0, help='Optional frame cap for quick checks')
    args = parser.parse_args()

    report = validate_session(args.session, threshold=args.threshold, max_frames=args.max_frames)
    _print_report(report)


if __name__ == '__main__':
    main()
