# Stereo Coordinate & Kinematics Convention (VS2)

This project now uses a locked, right-handed world coordinate convention.

## 1) Global World Frame W

- +X: Right
- +Y: Up
- +Z: Forward (away from front camera)

## 2) Camera Frame (OpenCV)

- +X: Right in image
- +Y: Down in image
- +Z: Forward from lens

`front camera` is treated as world origin through calibration/triangulation.
An explicit axis transform maps camera-style coordinates to world convention (Y-up).

## 3) Body Segment Convention

- Upper Arm (R): `Elbow - Shoulder`
- Forearm (R): `Wrist - Elbow`
- Trunk axis: `MidShoulder - MidHip`
- Pelvic axis: `RightHip - LeftHip`

## 4) Angle Convention

- Unit: degrees
- Range: 0° to 180°
- Example (Right Elbow):
  - `v1 = Shoulder - Elbow`
  - `v2 = Wrist - Elbow`
  - `theta = arccos((v1·v2) / (|v1||v2|))`

Interpretation:
- 180°: full extension
- lower values: increasing flexion

## 5) End-to-End Workflow

```
Capture (Front)
Capture (Right)
       │
       ▼
   Time Sync
       │
       ▼
Matched Frame Pair
       │
       ▼
  Triangulate
       │
       ▼
 3D Frame Object
       │
       ▼
  Kinematics
       │
       ▼
    Store
       │
       ▼
   Display
```

- **Capture (Front/Right):** Each camera captures frame + pose landmarks with timestamp.
- **Time Sync:** Master aligns timestamps within `SYNC_TIME_THRESHOLD_MS`.
- **Matched Frame Pair:** Only synchronized Front/Right frames continue to 3D.
- **Triangulate:** Uses calibrated projection matrices and distortion compensation.
- **3D Frame Object:** Structured fused payload for one synchronized instant (3D pose + quality metadata).
- **Kinematics:** Computes joint velocity, acceleration, angles, and angular velocity.
- **Store:** Persists synchronized 2D/3D and derived metrics.
- **Display:** Renders dual view + 3D/metric outputs in GUI/dashboard.

## 6) Validation Protocol

- Static jitter test (20s standing): report per-joint variance in XYZ
- Known-angle test (e.g., 90° elbow): compare measured `Angle_Elbow_R/L`
- Known-distance test (e.g., 1m depth check): compare triangulated `Z` against physical distance

# VS5 Motion Capture - Multi-Camera Enhanced Edition

Real-time multi-person pose, face, and hand tracking with **dual-laptop stereo 3D reconstruction**.

![Python](https://img.shields.io/badge/python-3.8+-blue.svg)
![MediaPipe](https://img.shields.io/badge/MediaPipe-0.10.9-green.svg)
![OpenCV](https://img.shields.io/badge/OpenCV-4.8+-red.svg)

---

## 🌟 Features

### Core Motion Capture
- **Multi-Person Tracking**: Detect up to 5 people simultaneously
- **Full Body Analysis**: 33 pose landmarks, 468 face landmarks, 21 hand landmarks per hand
- **Real-time Processing**: 25-30 FPS on modern hardware
- **Physics-Based Correction**: 1-Euro filtering and skeletal constraints for realistic motion
- **Comprehensive Metrics**: Joint angles, limb lengths, velocities, kinematics

### NEW: Multi-Camera System ✨
- **Stereo 3D Reconstruction**: Use 2+ laptops for triangulated 3D poses
- **Improved Accuracy**: ±1-2cm precision (vs ±5-10cm single camera)
- **Network Synchronization**: Frame-level timestamp matching
- **Occlusion Handling**: One camera sees what the other misses
- **Metric-Scale Depth**: True distance measurements (meters)

### Advanced Features
- **ROI-Based Detection**: Crop to person for higher effective resolution
- **Image Preprocessing**: Gamma correction, CLAHE, face-guided exposure
- **Dual Interfaces**: Desktop GUI (Tkinter) and Web interface (React)
- **Session Recording**: SQLite/PostgreSQL database with CSV export
- **Automated Reports**: Generated biomechanical analysis with Plotly visualizations
- **3D Visualization**: Interactive skeleton playback

---

## 📋 Requirements

- **OS**: Windows / macOS / Linux
- **Python**: 3.8 - 3.10
- **Webcam**: Built-in or USB camera (1280x720 recommended)
- **RAM**: 4GB minimum, 8GB recommended
- **For Multi-Camera**: 2+ laptops on the same WiFi network

---

## 🚀 Quick Start

### Single Camera Mode

```bash
# Clone repository
git clone https://github.com/Mrudula-itsjuzme/Motion-capture.git
cd Motion-capture/vs5

# Create virtual environment
python -m venv venv

# Activate (Windows)
.\venv\Scripts\Activate.ps1
# Or (Mac/Linux)
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Run GUI application
python main_gui.py
```

### Multi-Camera Mode (2 Laptops)

**Laptop 1 (Server)**:
```bash
python launch_multi_camera.py --mode server
```

**Laptop 2 (Master)**:
```bash
# Replace with Laptop 1's IP address
python launch_multi_camera.py --mode master --remote-ip <SERVER_IP>
# Alias also supported
python launch_multi_camera.py --mode master --remote <SERVER_IP>
```

See [docs/SETUP.md](docs/SETUP.md) for detailed multi-camera setup.

---

## 📁 Project Structure

```
vs5/
├── config.py                  # Central configuration
├── main_gui.py                # Desktop GUI (Tkinter)
├── launch_multi_camera.py     # Compatibility launcher (delegates to tools/)
├── requirements.txt
├── CHANGES.md                 # Cross-system change log (Windows/Mac sync)
├── SYSTEM_ARCHITECTURE.md     # Implementation-accurate architecture
│
├── tools/                     # Utility launchers & scripts
│   └── launch_multi_camera.py # Primary multi-camera launcher
│
├── docs/                      # Documentation
│   ├── SETUP.md
│   ├── DOCUMENTATION.md
│   ├── IMPLEMENTATION_PROGRESS.md
│   ├── UPDATE_NOTES.md
│   └── COORDINATE_SYSTEM_SPEC.md
│
├── data/                      # Sample data & generated dashboards
│   ├── mocap_*.csv
│   ├── mocap_data.db
│   └── dashboard_session_*.html
│
├── src/                       # Core modules
│   ├── camera.py              # Camera capture
│   ├── detector.py            # MediaPipe detection
│   ├── pose_corrector.py      # Physics-based correction
│   ├── calculations.py        # Biomechanical metrics
│   ├── visualizer.py          # 2D rendering
│   ├── visualizer_3d.py       # 3D Plotly visualization
│   ├── live_visualizer_3d.py  # Real-time 3D display
│   ├── database.py            # Session recording
│   ├── report_generator.py    # Automated reports
│   ├── streamer.py            # Video streaming
│   ├── one_euro_filter.py     # Adaptive smoothing
│   ├── camera_server.py       # Network broadcaster
│   ├── master_coordinator.py  # Frame aggregator
│   ├── frame_synchronizer.py  # Timestamp matching
│   ├── stereo_calibration.py  # Camera calibration
│   └── triangulation.py       # 3D reconstruction
│
├── scripts/                   # Helper scripts
│   ├── run_gui.bat            # Launch single-camera GUI
│   ├── run_master.bat         # Launch master mode
│   └── allow_firewall.ps1     # Open network ports
│
├── tests/                     # Network & integration tests
│   ├── test_connection.py
│   ├── test_multicam.py
│   ├── test_ports.py
│   ├── test_remote_ports.py
│   ├── test_receive_debug.py
│   ├── test_sender.py
│   └── test_sender_continuous.py
│
├── models/                    # MediaPipe model files
├── frontend/                  # React web interface
├── output/                    # Generated dashboards & exports
└── results/                   # Generated report images
```

---

## 🎯 Usage

### Configuration

Edit `config.py` to customize:

```python
# Camera
CAMERA_ID = 0
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720

# Detection
POSE_MODEL_COMPLEXITY = 'FULL'  # LITE, FULL, or HEAVY
NUM_POSES = 5  # Max people to track

# Multi-Camera
ENABLE_MULTI_CAMERA = True
CAMERA_ROLE = 'server'  # 'single', 'server', or 'master'
```

### Desktop GUI

```bash
python main_gui.py
```

Features:
- Live video preview with landmark overlay
- Real-time metrics display
- Recording start/stop
- Model complexity selector
- Image preprocessing controls
- Session export
- 3D visualization launcher




---

## 📊 Multi-Camera System

### Architecture

```
Laptop 1 (Server)         Laptop 2 (Master + Server)
─────────────────         ─────────────────────────
Camera → Detector    →    MasterCoordinator
     ↓                         ↓
CameraServer         →    FrameSynchronizer
(Port 5000-5001)              ↓
                          Triangulator
                              ↓
                          Unified 3D Pose
```

### Setup Steps

1. **Find IP Addresses**: Run `ipconfig` (Windows) or `ifconfig` (Mac/Linux)
2. **Open Firewall**: Run `scripts\allow_firewall.ps1` as Admin on Server PC
3. **Run Server**: `python launch_multi_camera.py --mode server`
4. **Run Master**: `python launch_multi_camera.py --mode master --remote-ip <SERVER_IP>`
5. **Calibrate** (optional): Use checkerboard pattern for accurate 3D

> See [docs/SETUP.md](docs/SETUP.md) for full step-by-step instructions.

### Expected Improvements

| Metric | Single Camera | Dual Camera |
|--------|--------------|-------------|
| 3D Accuracy | ±5-10cm | ±1-2cm |
| Depth Quality | Approximate | Metric-scale |
| Occlusion Handling | Poor | Good |
| Confidence Score | 0.5-0.8 | 0.7-0.95 |

---

## 🔬 Algorithms & Technologies

- **Detection**: MediaPipe Pose/Face/Hand Landmarker
- **Smoothing**: 1-Euro Filter (adaptive low-pass)
- **Correction**: Bone length constraints, joint limits
- **Triangulation**: Direct Linear Transform (DLT)
- **Synchronization**: Timestamp-based frame matching
- **Database**: SQLite (default) or PostgreSQL

---

## 📈 Performance

- **Single Camera**: 25-30 FPS @ 1280x720
- **Multi-Camera**: 15-25 FPS (network overhead)
- **Memory**: ~500 MB (FULL model)
- **CPU**: 40-60% (quad-core)

Optimizations:
- ROI cropping reduces inference cost ~40%
- Frame skipping configurable
- LITE model achieves 60+ FPS

---

## 🛠️ Development

### Running Tests

```bash
# Network layer
python -m src.camera_server
python -m src.master_coordinator

# Frame synchronization
python -m src.frame_synchronizer

# Triangulation
python -m src.triangulation
```

### Adding New Features

See `implementation_plan.md` for architecture details.

---

## 📚 Documentation

- **Setup Guide**: [docs/SETUP.md](docs/SETUP.md) - Detailed dual-PC setup instructions
- **Configuration**: [config.py](config.py) - All tunable parameters

### Data Verification (Multi-Camera)

After recording + CSV export, `Source` should include:
- `PC1` (local camera on master)
- `PC2` (remote Windows/server camera)
- `3D` (triangulated landmarks)
- `KIN` (flattened kinematics metrics)

If `PC2` is missing in CSV, first restart both server and master processes (to ensure both run latest code), then record a new short session.

### SQLite Viewer (VS Code)

Install **SQLite Viewer** extension (`qwtel.sqlite-viewer`) and open `data/mocap_data.db` from Explorer.

---

## 🎓 Use Cases

- Biomechanical research and analysis
- Physical therapy assessment
- Sports performance analysis
- Animation reference capture
- Human-computer interaction research
- Fitness and exercise tracking

---

## 🚧 Known Issues

- Calibration wizard in development
- Multi-person association across cameras needs work
- Network latency varies with WiFi quality

---

## 🔜 Roadmap

- [ ] GUI integration for multi-camera mode
- [ ] Interactive calibration wizard
- [ ] Real-time 3D visualization in GUI
- [ ] BVH/FBX export for animation software
- [ ] GPU acceleration (CUDA)
- [ ] Cloud recording with PostgreSQL
- [ ] 4+ camera support

---

## 📝 License

MIT License - See LICENSE file for details

---

## 🙏 Acknowledgments

- **MediaPipe** by Google for pose/face/hand detection
- **OpenCV** for computer vision utilities
- **ZeroMQ** for high-performance networking

---

## 📧 Contact

**Author**: Mrudula  
**GitHub**: [@Mrudula-itsjuzme](https://github.com/Mrudula-itsjuzme)  
**Repository**: [Motion-capture](https://github.com/Mrudula-itsjuzme/Motion-capture)

---

## 🌟 Star This Repo!

If you find this project useful, please give it a star ⭐ on GitHub!

---

**Built with ❤️ for the motion capture community**
