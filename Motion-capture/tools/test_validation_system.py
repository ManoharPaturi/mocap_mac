#!/usr/bin/env python3
"""
Smoke checks for 4-layer validation stack.
"""

import os
import sys

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from src.database import MocapDB  # noqa: E402


REQUIRED_TABLES = {
    'frames',
    'raw_landmarks_2d',
    'joints_3d',
    'kinematics_3d',
    'kinematics_2d',
    'velocities',
    'accelerations',
    'validation_runs',
    'validation_angle_comparison',
    'validation_artifacts',
}

REQUIRED_METHODS = [
    'save_frame_metadata',
    'save_raw_landmarks',
    'save_joints_3d',
    'save_kinematics_3d',
    'save_validation_run',
    'get_session_frames',
    'get_joints_3d_for_frame',
    'get_kinematics_3d_for_frame',
    'get_raw_landmarks_for_frame',
]


def _list_tables(db: MocapDB):
    conn = db._get_connection()
    cur = conn.cursor()
    if db.db_type == 'postgres':
        cur.execute("SELECT tablename FROM pg_tables WHERE schemaname='public'")
    else:
        cur.execute("SELECT name FROM sqlite_master WHERE type='table'")
    rows = cur.fetchall()
    conn.close()
    return {str(r[0]) for r in rows}


def main() -> None:
    db = MocapDB()

    missing_methods = [m for m in REQUIRED_METHODS if not hasattr(db, m)]
    if missing_methods:
        raise RuntimeError(f"Missing required DB methods: {missing_methods}")

    tables = _list_tables(db)
    missing_tables = sorted(REQUIRED_TABLES - tables)
    if missing_tables:
        raise RuntimeError(f"Missing required tables: {missing_tables}")

    print('✅ Validation system smoke test passed')
    print(f'   DB Type: {db.db_type}')
    print(f'   Required methods: {len(REQUIRED_METHODS)} present')
    print(f'   Required tables: {len(REQUIRED_TABLES)} present')


if __name__ == '__main__':
    main()
