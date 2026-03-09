#!/usr/bin/env python3
"""Offline stereo calibration from chessboard image folders.

Outputs:
- calibration JSON (system format)
- intrinsics_front.yaml
- intrinsics_right.yaml
- stereo_extrinsics.yaml

Enforces strict acceptance: RMS reprojection error must be < threshold (default 1.0 px).
"""

import argparse
import os
import sys
from pathlib import Path
from typing import List

import cv2

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.stereo_calibration import StereoCalibration


def load_images(folder: Path) -> List:
    patterns = ("*.png", "*.jpg", "*.jpeg", "*.bmp")
    files = []
    for pattern in patterns:
        files.extend(sorted(folder.glob(pattern)))
    images = []
    for file_path in files:
        image = cv2.imread(str(file_path))
        if image is not None:
            images.append(image)
    return images


def main() -> int:
    parser = argparse.ArgumentParser(description="Calibrate stereo cameras from chessboard image folders")
    parser.add_argument("--front-dir", required=True, help="Folder with front camera chessboard images")
    parser.add_argument("--right-dir", required=True, help="Folder with right camera chessboard images")
    parser.add_argument("--board-cols", type=int, default=9, help="Checkerboard inner corners (columns)")
    parser.add_argument("--board-rows", type=int, default=6, help="Checkerboard inner corners (rows)")
    parser.add_argument("--square-mm", type=float, default=25.0, help="Checkerboard square size in mm")
    parser.add_argument("--threshold-px", type=float, default=1.0, help="Acceptance threshold for stereo RMS")
    parser.add_argument("--output-json", default="calibration.json", help="Output calibration JSON path")
    parser.add_argument("--output-dir", default=".", help="Directory for YAML exports")
    parser.add_argument("--front-camera-id", default="cam_0", help="Camera ID for front camera in JSON")
    parser.add_argument("--right-camera-id", default="cam_1", help="Camera ID for right camera in JSON")
    args = parser.parse_args()

    front_dir = Path(args.front_dir)
    right_dir = Path(args.right_dir)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    if not front_dir.exists() or not right_dir.exists():
        print("[CalibrationCLI] Input folder does not exist")
        return 2

    front_images = load_images(front_dir)
    right_images = load_images(right_dir)

    if len(front_images) == 0 or len(right_images) == 0:
        print("[CalibrationCLI] No images found in one or both folders")
        return 2

    pair_count = min(len(front_images), len(right_images))
    front_images = front_images[:pair_count]
    right_images = right_images[:pair_count]

    print(f"[CalibrationCLI] Loaded {pair_count} stereo pairs")

    calibrator = StereoCalibration()
    checkerboard = (args.board_cols, args.board_rows)
    square_m = args.square_mm / 1000.0

    try:
        cal_front, cal_right = calibrator.calibrate_stereo(
            args.front_camera_id,
            args.right_camera_id,
            front_images,
            right_images,
            checkerboard_size=checkerboard,
            square_size=square_m,
        )
    except Exception as exc:
        print(f"[CalibrationCLI] Calibration failed: {exc}")
        return 1

    stereo_rms = calibrator.metadata.get("rms_error")
    if stereo_rms is None:
        stereo_rms = (float(cal_front.reprojection_error) + float(cal_right.reprojection_error)) / 2.0
        print(f"[CalibrationCLI] Stereo RMS unavailable, fallback intrinsic mean: {stereo_rms:.4f}px")
    else:
        stereo_rms = float(stereo_rms)
        print(f"[CalibrationCLI] Stereo RMS reprojection error: {stereo_rms:.4f}px")

    if stereo_rms >= args.threshold_px:
        print(f"[CalibrationCLI] Rejected: error {stereo_rms:.4f}px >= {args.threshold_px:.4f}px")
        return 3

    calibrator.save_calibration(args.output_json)

    front_yaml = output_dir / "intrinsics_front.yaml"
    right_yaml = output_dir / "intrinsics_right.yaml"
    stereo_yaml = output_dir / "stereo_extrinsics.yaml"

    calibrator.save_intrinsics_yaml(args.front_camera_id, str(front_yaml))
    calibrator.save_intrinsics_yaml(args.right_camera_id, str(right_yaml))
    calibrator.save_stereo_yaml(args.front_camera_id, args.right_camera_id, str(stereo_yaml))

    print(f"[CalibrationCLI] Saved JSON: {args.output_json}")
    print(f"[CalibrationCLI] Saved YAML: {front_yaml}")
    print(f"[CalibrationCLI] Saved YAML: {right_yaml}")
    print(f"[CalibrationCLI] Saved YAML: {stereo_yaml}")
    print("[CalibrationCLI] Calibration accepted (< threshold)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
