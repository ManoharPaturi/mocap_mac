#!/usr/bin/env python3
"""
Export 3D/2D/validation artifacts from the layered mocap database.
"""

import argparse
import csv
import json
import os
import sys
from collections import defaultdict
from datetime import datetime
from typing import Any, Dict, List, Optional

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from src.database import MocapDB  # noqa: E402


def _latest_validation_run_id(db: MocapDB, session_id: str) -> Optional[int]:
    conn = db._get_connection()
    cur = conn.cursor()
    ph = '%s' if db.db_type == 'postgres' else '?'
    cur.execute(
        f"SELECT id FROM validation_runs WHERE session_id = {ph} ORDER BY id DESC LIMIT 1",
        (session_id,),
    )
    row = cur.fetchone()
    conn.close()
    return int(row[0]) if row else None


def _record_artifact(db: MocapDB, validation_run_id: Optional[int], session_id: str, artifact_type: str, file_path: str, metadata: Dict[str, Any]) -> None:
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
            file_path,
            json.dumps(metadata),
            datetime.now().isoformat(),
        ),
    )
    conn.commit()
    conn.close()


def _export_3d_joints(db: MocapDB, session_id: str, output_dir: str) -> str:
    conn = db._get_connection()
    cur = conn.cursor()
    ph = '%s' if db.db_type == 'postgres' else '?'
    cur.execute(
        f"SELECT frame_id, joint_id, x, y, z, confidence, reprojection_error_px, timestamp_ms, triangulation_version, algorithm "
        f"FROM joints_3d WHERE session_id = {ph} ORDER BY frame_id, joint_id",
        (session_id,),
    )
    rows = cur.fetchall()
    conn.close()

    out_path = os.path.join(output_dir, f'3d_coordinates_{session_id}.csv')
    with open(out_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow([
            'frame_id', 'joint_id', 'x', 'y', 'z',
            'confidence', 'reprojection_error_px', 'timestamp_ms',
            'triangulation_version', 'algorithm',
        ])
        writer.writerows(rows)

    return out_path


def _export_2d_landmarks(db: MocapDB, session_id: str, output_dir: str) -> List[str]:
    conn = db._get_connection()
    cur = conn.cursor()
    ph = '%s' if db.db_type == 'postgres' else '?'

    cur.execute(
        f"SELECT DISTINCT camera_id FROM raw_landmarks_2d WHERE session_id = {ph} ORDER BY camera_id",
        (session_id,),
    )
    camera_ids = [str(row[0]) for row in cur.fetchall()]

    exported = []
    for camera_id in camera_ids:
        cur.execute(
            f"SELECT frame_id, landmark_id, x, y, z, visibility, timestamp_ms, source "
            f"FROM raw_landmarks_2d WHERE session_id = {ph} AND camera_id = {ph} ORDER BY frame_id, landmark_id",
            (session_id, camera_id),
        )
        rows = cur.fetchall()

        safe_cam = camera_id.replace('/', '_').replace(':', '_')
        out_path = os.path.join(output_dir, f'landmarks_{safe_cam}_{session_id}.csv')
        with open(out_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(['frame_id', 'landmark_id', 'x', 'y', 'z', 'visibility', 'timestamp_ms', 'source'])
            writer.writerows(rows)

        exported.append(out_path)

    conn.close()
    return exported


def _export_angle_comparison(db: MocapDB, run_id: int, output_dir: str, session_id: str) -> str:
    conn = db._get_connection()
    cur = conn.cursor()
    ph = '%s' if db.db_type == 'postgres' else '?'
    cur.execute(
        f"SELECT frame_id, angle_name, realtime_angle, offline_angle, error_deg, pass_flag "
        f"FROM validation_angle_comparison WHERE validation_run_id = {ph} ORDER BY frame_id, angle_name",
        (run_id,),
    )
    rows = cur.fetchall()
    conn.close()

    out_path = os.path.join(output_dir, f'angle_comparison_{session_id}.csv')
    with open(out_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(['frame_id', 'angle_name', 'realtime_angle', 'offline_angle', 'error_deg', 'pass_flag'])
        writer.writerows(rows)

    return out_path


def _export_session_metadata(db: MocapDB, session_id: str, output_dir: str) -> str:
    conn = db._get_connection()
    cur = conn.cursor()
    ph = '%s' if db.db_type == 'postgres' else '?'

    cur.execute(
        f"SELECT COUNT(*) FROM frames WHERE session_id = {ph}",
        (session_id,),
    )
    num_frames = int(cur.fetchone()[0])

    cur.execute(
        f"SELECT COUNT(*) FROM raw_landmarks_2d WHERE session_id = {ph}",
        (session_id,),
    )
    num_raw = int(cur.fetchone()[0])

    cur.execute(
        f"SELECT COUNT(*) FROM joints_3d WHERE session_id = {ph}",
        (session_id,),
    )
    num_joints = int(cur.fetchone()[0])

    cur.execute(
        f"SELECT COUNT(*) FROM kinematics_3d WHERE session_id = {ph}",
        (session_id,),
    )
    num_kin = int(cur.fetchone()[0])

    cur.execute(
        f"SELECT recording_mode, start_time FROM sessions WHERE id = {ph}",
        (session_id,),
    )
    session_row = cur.fetchone()
    conn.close()

    metadata = {
        'session_id': session_id,
        'recording_mode': session_row[0] if session_row else None,
        'start_time': session_row[1] if session_row else None,
        'frames': num_frames,
        'raw_landmarks_2d_rows': num_raw,
        'joints_3d_rows': num_joints,
        'kinematics_3d_rows': num_kin,
        'generated_at': datetime.now().isoformat(),
    }

    out_path = os.path.join(output_dir, f'session_metadata_{session_id}.json')
    with open(out_path, 'w', encoding='utf-8') as f:
        json.dump(metadata, f, indent=2)

    return out_path


def main() -> None:
    parser = argparse.ArgumentParser(description='Export validation and layered data artifacts.')
    parser.add_argument('--session', required=True, help='Session ID')
    parser.add_argument('--output-dir', default=os.path.join('results', 'validation'))
    parser.add_argument('--include-2d', action='store_true', help='Include Layer-1 per-camera landmarks CSV')
    parser.add_argument('--include-all', action='store_true', help='Export all available artifacts (includes 2D + comparisons + metadata)')
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    db = MocapDB()
    run_id = _latest_validation_run_id(db, args.session)

    export_2d = args.include_2d or args.include_all
    export_all = args.include_all

    generated = []

    joints_path = _export_3d_joints(db, args.session, args.output_dir)
    generated.append(('joints_3d_csv', joints_path, {'session_id': args.session}))

    if export_2d:
        for path in _export_2d_landmarks(db, args.session, args.output_dir):
            generated.append(('raw_landmarks_2d_csv', path, {'session_id': args.session}))

    if export_all and run_id is not None:
        compare_path = _export_angle_comparison(db, run_id, args.output_dir, args.session)
        generated.append(('validation_angle_comparison_csv', compare_path, {'session_id': args.session, 'validation_run_id': run_id}))

    if export_all:
        meta_path = _export_session_metadata(db, args.session, args.output_dir)
        generated.append(('session_metadata_json', meta_path, {'session_id': args.session}))

    for artifact_type, path, metadata in generated:
        _record_artifact(db, run_id, args.session, artifact_type, path, metadata)

    print(f"Exported {len(generated)} artifact(s) for session {args.session}:")
    for _, path, _ in generated:
        print(f" - {path}")


if __name__ == '__main__':
    main()
