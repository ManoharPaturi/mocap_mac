# VS7.1 Motion Capture — Multi-Camera Stereo Edition

Real-time, multi-person **markerless motion capture** (pose + face + hands) with
**dual-laptop stereo 3D reconstruction**: one machine runs camera servers streaming MediaPipe
landmarks over ZeroMQ; a master machine timestamp-synchronizes frames, undistorts them,
**DLT-triangulates** 3D joints, filters with a 1-Euro filter, computes live kinematics (joint
angles, velocities, accelerations), renders a live 3D dashboard, and records every session to
SQLite/Postgres — with a validation pipeline (jitter, known-angle, known-distance tests) that
produces Plotly reports.

```text
laptop 1 (server)                          laptop 2 (master)
 camera ─▶ MediaPipe pose/face/hands ─ZMQ─▶ frame sync (±20 ms) ─▶ undistort ─▶ DLT triangulate
                                           ─▶ 1-Euro filter ─▶ kinematics engine
                                           ─▶ live 3D dashboard + SQLite/Postgres logging
                                           ─▶ evaluation pipeline ─▶ Plotly reports
```

- **Multi-person** — up to 5 people; 33 body landmarks each, plus 468 face points and 21 per hand
- **Locked right-handed world frame** — Y-up, front camera as origin (full convention in
  [`Motion-capture/README.md`](Motion-capture/README.md) and
  [`Motion-capture/docs/COORDINATE_SYSTEM_SPEC.md`](Motion-capture/docs/COORDINATE_SYSTEM_SPEC.md))
- **Claimed accuracy ±1–2 cm at 25–30 FPS** on the stereo reconstruction, verified by the
  validation suite in `Motion-capture/tools/`

## Quick start

```bash
cd Motion-capture
python -m venv venv && source venv/bin/activate
pip install -r requirements.txt

python main_gui.py                    # single-camera GUI
python launch_multi_camera.py --mode server        # laptop 1
python launch_multi_camera.py --mode master --remote-ip <SERVER_IP>   # laptop 2
```

Web frontend: `cd frontend && npm install && npm run dev`.

## Validation

```bash
python tools/validate_session.py --session <SESSION_ID> --threshold 2.0
```

See [`Motion-capture/tools/VALIDATION_GUIDE.md`](Motion-capture/tools/VALIDATION_GUIDE.md) for
the full protocol (stereo calibration from images, angle comparison, artifact export) and
`results.md` for consolidated metrics.

## Stack

Python 3.8–3.10 · OpenCV · MediaPipe 0.10.9 · PyTorch (CUDA/MPS) · ZeroMQ + msgpack ·
FastAPI · pandas/Plotly · Tkinter desktop GUI · React + Vite web frontend

## Repo map

| Path | Purpose |
|---|---|
| `Motion-capture/main_gui.py` | Desktop GUI (master/server modes) |
| `Motion-capture/src/` | ~30 modules: coordinator, synchronizer, calibration, triangulation, kinematics, filtering, DB, evaluation pipeline |
| `Motion-capture/tools/` | Validation suite + calibration + comparison scripts |
| `Motion-capture/frontend/` | React web dashboard |
| `Motion-capture/docs/` | Setup, DB schema, coordinate system, architecture |

Earlier stages of this system live in the [`mocap`](https://github.com/ManoharPaturi/mocap)
repo (VS1→VS5 history); this repo is the macOS line at VS6→VS7.1 with the evaluation pipeline
and async DB writes.
