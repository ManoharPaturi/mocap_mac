#!/usr/bin/env python3
"""
Generate detailed reports from Layer-4 validation results.
"""

import argparse
import csv
import json
import os
import sys
from collections import defaultdict
from datetime import datetime
from typing import Any, Dict, List

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from src.database import MocapDB  # noqa: E402


def _latest_validation_run_id(db: MocapDB, session_id: str) -> int:
    conn = db._get_connection()
    cur = conn.cursor()
    ph = '%s' if db.db_type == 'postgres' else '?'
    cur.execute(
        f"SELECT id FROM validation_runs WHERE session_id = {ph} ORDER BY id DESC LIMIT 1",
        (session_id,),
    )
    row = cur.fetchone()
    conn.close()
    if not row:
        raise RuntimeError(f"No validation_runs found for session {session_id}. Run tools/validate_session.py first.")
    return int(row[0])


def _load_rows(db: MocapDB, validation_run_id: int) -> List[Dict[str, Any]]:
    conn = db._get_connection()
    cur = conn.cursor()
    ph = '%s' if db.db_type == 'postgres' else '?'
    cur.execute(
        f"SELECT frame_id, angle_name, realtime_angle, offline_angle, error_deg, pass_flag "
        f"FROM validation_angle_comparison WHERE validation_run_id = {ph}",
        (validation_run_id,),
    )
    fetched = cur.fetchall()
    conn.close()

    rows = []
    for frame_id, angle_name, realtime_angle, offline_angle, error_deg, pass_flag in fetched:
        rows.append(
            {
                'frame_id': int(frame_id),
                'angle_name': str(angle_name),
                'realtime_angle': float(realtime_angle),
                'offline_angle': float(offline_angle),
                'error_deg': float(error_deg),
                'pass_flag': int(pass_flag),
            }
        )
    return rows


def _record_artifact(db: MocapDB, validation_run_id: int, session_id: str, artifact_type: str, file_path: str, metadata: Dict[str, Any]) -> None:
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


def _stats(rows: List[Dict[str, Any]]) -> Dict[str, Any]:
    total = len(rows)
    passed = sum(r['pass_flag'] for r in rows)
    errors = [r['error_deg'] for r in rows]

    per_angle = defaultdict(list)
    for row in rows:
        per_angle[row['angle_name']].append(row['error_deg'])

    return {
        'total_rows': total,
        'pass_rate': (passed / total * 100.0) if total else 0.0,
        'mean_error': (sum(errors) / total) if total else 0.0,
        'max_error': max(errors) if errors else 0.0,
        'angles': {
            name: {
                'count': len(vals),
                'mean_error': sum(vals) / len(vals),
                'max_error': max(vals),
            }
            for name, vals in sorted(per_angle.items())
        },
    }


def _write_csv(path: str, rows: List[Dict[str, Any]]) -> None:
    with open(path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(
            f,
            fieldnames=['frame_id', 'angle_name', 'realtime_angle', 'offline_angle', 'error_deg', 'pass_flag'],
        )
        writer.writeheader()
        writer.writerows(rows)


def _write_markdown(path: str, session_id: str, run_id: int, stats: Dict[str, Any], rows: List[Dict[str, Any]]) -> None:
    worst = sorted(rows, key=lambda r: r['error_deg'], reverse=True)[:20]

    lines = [
        '# Angle Comparison Report',
        '',
        f'- Session: {session_id}',
        f'- Validation Run ID: {run_id}',
        f"- Generated: {datetime.now().isoformat()}",
        f"- Pass Rate: {stats['pass_rate']:.2f}%",
        f"- Mean Error: {stats['mean_error']:.3f}°",
        f"- Max Error: {stats['max_error']:.3f}°",
        '',
        '## Worst 20 Errors',
        '',
        '| Frame | Angle | Realtime | Offline | Error | Pass |',
        '|---:|---|---:|---:|---:|---:|',
    ]

    for row in worst:
        lines.append(
            f"| {row['frame_id']} | {row['angle_name']} | {row['realtime_angle']:.3f} | "
            f"{row['offline_angle']:.3f} | {row['error_deg']:.3f} | {row['pass_flag']} |"
        )

    lines.extend(['', '## Per-Angle Statistics', '', '| Angle | Count | Mean Error | Max Error |', '|---|---:|---:|---:|'])
    for name, detail in stats['angles'].items():
        lines.append(
            f"| {name} | {detail['count']} | {detail['mean_error']:.3f} | {detail['max_error']:.3f} |"
        )

    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines))


def main() -> None:
    parser = argparse.ArgumentParser(description='Generate reports from validation_angle_comparison.')
    parser.add_argument('--session', required=True, help='Session ID')
    parser.add_argument('--format', default='all', choices=['csv', 'markdown', 'json', 'all'])
    parser.add_argument('--output-dir', default=os.path.join('results', 'validation'))
    args = parser.parse_args()

    db = MocapDB()
    run_id = _latest_validation_run_id(db, args.session)
    rows = _load_rows(db, run_id)
    if not rows:
        raise RuntimeError(f"No validation comparison rows found for run {run_id}")

    os.makedirs(args.output_dir, exist_ok=True)
    stats = _stats(rows)

    generated = []
    if args.format in ('csv', 'all'):
        path = os.path.join(args.output_dir, f'comparison_{args.session}.csv')
        _write_csv(path, rows)
        generated.append(('comparison_csv', path))

    if args.format in ('markdown', 'all'):
        path = os.path.join(args.output_dir, f'report_{args.session}.md')
        _write_markdown(path, args.session, run_id, stats, rows)
        generated.append(('comparison_markdown', path))

    if args.format in ('json', 'all'):
        path = os.path.join(args.output_dir, f'stats_{args.session}.json')
        with open(path, 'w', encoding='utf-8') as f:
            json.dump({'session_id': args.session, 'validation_run_id': run_id, 'stats': stats}, f, indent=2)
        generated.append(('comparison_stats_json', path))

    for artifact_type, path in generated:
        _record_artifact(
            db,
            run_id,
            args.session,
            artifact_type=artifact_type,
            file_path=path,
            metadata={'rows': len(rows)},
        )

    print(f"Generated {len(generated)} artifact(s) for session {args.session} (run {run_id})")
    for _, path in generated:
        print(f" - {path}")


if __name__ == '__main__':
    main()
