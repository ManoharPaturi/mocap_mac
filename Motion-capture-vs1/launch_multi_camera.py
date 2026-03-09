"""
Multi-Camera Launcher
Easily switch between single, server, and master modes
"""

import sys
import os
import argparse

# Ensure the project root is on sys.path and is the working directory so that
# `import config` and `from src.xxx import ...` resolve correctly regardless
# of how this script is invoked.
_PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)
os.chdir(_PROJECT_ROOT)

# Suppress TFLite/MediaPipe/abseil noise before any imports
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'          # TensorFlow: errors only
os.environ['GLOG_minloglevel'] = '2'               # Google logging: warnings+ only
os.environ['ABSL_MIN_LOG_LEVEL'] = '2'             # abseil: warnings+ only
os.environ['MEDIAPIPE_DISABLE_GPU_LOG'] = '1'      # MediaPipe GPU logs

import logging
logging.getLogger('mediapipe').setLevel(logging.ERROR)
logging.getLogger('tensorflow').setLevel(logging.ERROR)
logging.getLogger('absl').setLevel(logging.ERROR)

# Parse arguments
parser = argparse.ArgumentParser(
    description='Launch VS5 Multi-Camera System',
    allow_abbrev=False
)
parser.add_argument('--mode', type=str, default='single',
                    choices=['single', 'server', 'master'],
                    help='Camera mode: single/server/master')
parser.add_argument('--remote-ip', '--remote', dest='remote_ip', type=str, default=None,
                    help='IP address of remote camera (for master mode)')
args = parser.parse_args()

# Update config dynamically
print("[LAUNCHER] Imports starting...")
try:
    import config
    print("[LAUNCHER] Config imported.")

    # Allow override of inference backend via environment variable
    _backend_override = os.environ.get('MOCAP_BACKEND_OVERRIDE', '').strip().lower()
    if _backend_override == 'cpu':
        config.INFERENCE_BACKEND = 'cpu'
        config.PREFER_GPU_DELEGATE = False
        print("[LAUNCHER] CPU backend override active (MOCAP_BACKEND_OVERRIDE=cpu)")

    # Update config dynamically
    config.MULTI_CAMERA_MODE = args.mode
    print(f"[LAUNCHER] Config mode set to: {args.mode}")

    if args.mode == 'master':
        if args.remote_ip:
            config.REMOTE_CAMERA_IP = args.remote_ip
            print(f"[LAUNCHER] Remote IP set to: {args.remote_ip}")
        else:
            print("WARNING: Master mode requires --remote-ip argument!")
            sys.exit(1)

    print("[LAUNCHER] Importing GUI...")
    from main_gui import MocapGUI
    print("[LAUNCHER] GUI imported.")

    print(f"\n{'='*60}")
    print(f"VS5 Motion Capture System")
    print(f"Mode: {args.mode.upper()}")
    if config.CUDA_ENABLED:
        accel_str = 'CUDA'
    elif config.MPS_ENABLED:
        accel_str = 'Metal (MPS)'
    else:
        accel_str = 'CPU'
    print(f"Hardware Acceleration: {accel_str}")
    if args.mode == 'master':
        print(f"Remote Camera: {config.REMOTE_CAMERA_IP}")
    print(f"{'='*60}\n")
    
    print("[LAUNCHER] Initializing App...")
    app = MocapGUI()
    print("[LAUNCHER] App Initialized. Starting Video Loop...")
    
    if hasattr(app, 'run'):
        app.run()
    elif hasattr(app, 'video_loop'):
        app.video_loop()
    else:
        print("[ERROR] No run/video_loop method found on MocapGUI!")
        
except Exception as e:
    print(f"\n[CRITICAL ERROR] Launcher failed: {e}")
    import traceback
    traceback.print_exc()
    input("Press Enter to exit...")
