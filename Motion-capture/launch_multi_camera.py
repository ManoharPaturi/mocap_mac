"""
Compatibility launcher shim.
Keeps `python launch_multi_camera.py ...` working after moving the real launcher
into `tools/launch_multi_camera.py` during repository cleanup.

Runs the real launcher as a subprocess so that a fatal MediaPipe GPU crash
(SIGABRT, exit 134) can be detected and the app automatically retried on CPU.
"""

import subprocess
import sys
import os
from pathlib import Path

if __name__ == "__main__":
    target = str(Path(__file__).resolve().parent / "tools" / "launch_multi_camera.py")
    env = os.environ.copy()

    # First attempt — use whatever backend is configured (MPS by default)
    result = subprocess.run([sys.executable, target] + sys.argv[1:], env=env)

    if result.returncode == 134:   # SIGABRT — MediaPipe fatal GPU/CVPixelBuffer crash
        print(
            "\n[LAUNCHER] ⚠️  GPU (MPS) crash detected (exit 134). "
            "Retrying with CPU backend ..."
        )
        env['MOCAP_BACKEND_OVERRIDE'] = 'cpu'
        result = subprocess.run([sys.executable, target] + sys.argv[1:], env=env)

    sys.exit(result.returncode)

