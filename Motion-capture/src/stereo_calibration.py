"""
Stereo Calibration Module
Handles intrinsic and extrinsic camera calibration for multi-view 3D reconstruction.
"""

import cv2
import numpy as np
import json
import os
import time
from typing import List, Tuple, Dict, Optional, Any
from dataclasses import dataclass
from config import (
    ARUCO_DICT_TYPE,
    ARUCO_BOARD_WIDTH,
    ARUCO_BOARD_HEIGHT,
    ARUCO_MARKER_LENGTH_M,
    ARUCO_MARKER_SEPARATION_M,
    ARUCO_MIN_IMAGES_INTRINSIC,
    ARUCO_MIN_IMAGES_STEREO,
    ARUCO_MIN_MARKERS_PER_IMAGE,
)


@dataclass
class CameraCalibration:
    """Calibration data for a single camera."""
    camera_id: str
    intrinsic_matrix: np.ndarray  # 3x3 camera matrix
    distortion_coeffs: np.ndarray  # (k1, k2, p1, p2, k3)
    rotation: Optional[np.ndarray] = None  # 3x3 rotation matrix (relative to world)
    translation: Optional[np.ndarray] = None  # 3x1 translation vector
    image_size: Tuple[int, int] = None  # (width, height)
    reprojection_error: float = 0.0


class StereoCalibration:
    """
    Handles camera calibration for multi-view motion capture.
    Supports both intrinsic (lens) and extrinsic (position/orientation) calibration.
    """
    
    def __init__(self):
        """Initialize stereo calibration system."""
        self.cameras: Dict[str, CameraCalibration] = {}
        self.metadata: Dict[str, Any] = {}
        self.calibration_criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30,
            0.001
        )
        print("[StereoCalibration] Initialized")
    
    def calibrate_intrinsic(
        self,
        camera_id: str,
        images: List[np.ndarray],
        checkerboard_size: Tuple[int, int] = (9, 6),
        square_size: float = 0.025  # meters
    ) -> CameraCalibration:
        """
        Calibrate intrinsic parameters (camera matrix, distortion) using checkerboard.
        
        Args:
            camera_id: Unique camera identifier
            images: List of grayscale/color images of checkerboard
            checkerboard_size: Number of internal corners (cols, rows)
            square_size: Size of checkerboard squares in meters
            
        Returns:
            CameraCalibration object with intrinsic parameters
        """
        print(f"[StereoCalibration] Calibrating intrinsics for {camera_id}...")
        
        # Prepare object points (3D points in real world space)
        objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[
            0:checkerboard_size[0],
            0:checkerboard_size[1]
        ].T.reshape(-1, 2)
        objp *= square_size
        
        # Arrays to store object points and image points
        obj_points = []  # 3D points in real world space
        img_points = []  # 2D points in image plane
        
        image_size = None
        found_count = 0
        
        for i, img in enumerate(images):
            if len(img.shape) == 3:
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            else:
                gray = img
            
            image_size = gray.shape[::-1]
            
            # Find checkerboard corners
            ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, None)
            
            if ret:
                obj_points.append(objp)
                
                # Refine corner positions
                corners_refined = cv2.cornerSubPix(
                    gray, corners, (11, 11), (-1, -1), self.calibration_criteria
                )
                img_points.append(corners_refined)
                found_count += 1
                print(f"  Found corners in image {i+1}/{len(images)}")
            else:
                print(f"  Failed to find corners in image {i+1}/{len(images)}")
        
        if found_count < 10:
            raise ValueError(
                f"Not enough valid calibration images. Found {found_count}, need at least 10"
            )
        
        print(f"  Successfully processed {found_count}/{len(images)} images")
        
        # Perform calibration
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            obj_points, img_points, image_size, None, None
        )
        
        # Calculate reprojection error
        mean_error = 0
        for i in range(len(obj_points)):
            img_points_reprojected, _ = cv2.projectPoints(
                obj_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs
            )
            error = cv2.norm(img_points[i], img_points_reprojected, cv2.NORM_L2) / len(
                img_points_reprojected
            )
            mean_error += error
        
        mean_error /= len(obj_points)
        
        print(f"  Calibration complete. Reprojection error: {mean_error:.3f} pixels")
        
        # Create calibration object
        calibration = CameraCalibration(
            camera_id=camera_id,
            intrinsic_matrix=camera_matrix,
            distortion_coeffs=dist_coeffs,
            image_size=image_size,
            reprojection_error=mean_error
        )
        
        self.cameras[camera_id] = calibration
        return calibration

    def _build_aruco_board(self,
                           dictionary_id: int = ARUCO_DICT_TYPE,
                           board_width: int = ARUCO_BOARD_WIDTH,
                           board_height: int = ARUCO_BOARD_HEIGHT,
                           marker_length_m: float = ARUCO_MARKER_LENGTH_M,
                           marker_separation_m: float = ARUCO_MARKER_SEPARATION_M):
        """Build an OpenCV ArUco GridBoard in a version-compatible way."""
        if not hasattr(cv2, 'aruco'):
            raise RuntimeError("OpenCV ArUco module not found. Install opencv-contrib-python.")

        dictionary = cv2.aruco.getPredefinedDictionary(int(dictionary_id))
        if hasattr(cv2.aruco, 'GridBoard'):
            board = cv2.aruco.GridBoard(
                (int(board_width), int(board_height)),
                float(marker_length_m),
                float(marker_separation_m),
                dictionary
            )
        elif hasattr(cv2.aruco, 'GridBoard_create'):
            board = cv2.aruco.GridBoard_create(
                int(board_width),
                int(board_height),
                float(marker_length_m),
                float(marker_separation_m),
                dictionary
            )
        else:
            raise RuntimeError("OpenCV ArUco GridBoard API unavailable in this OpenCV build.")

        return dictionary, board

    def generate_aruco_board_image(self,
                                   filepath: str,
                                   board_width: int = ARUCO_BOARD_WIDTH,
                                   board_height: int = ARUCO_BOARD_HEIGHT,
                                   marker_length_m: float = ARUCO_MARKER_LENGTH_M,
                                   marker_separation_m: float = ARUCO_MARKER_SEPARATION_M,
                                   dictionary_id: int = ARUCO_DICT_TYPE,
                                   dpi: int = 300) -> str:
        """Generate an ArUco board image for printing."""
        _, board = self._build_aruco_board(
            dictionary_id=dictionary_id,
            board_width=board_width,
            board_height=board_height,
            marker_length_m=marker_length_m,
            marker_separation_m=marker_separation_m,
        )

        # Compute canvas from integer pixel cell sizes so Python and OpenCV's
        # internal C++ rounding always agree, preventing the Mat ROI assertion
        # failure (matrix.cpp: roi.x + roi.width <= m.cols) seen in OpenCV 4.8+.
        pixels_per_meter = float(dpi) / 0.0254
        marker_px = max(10, int(float(marker_length_m) * pixels_per_meter))
        sep_px    = max(2,  int(float(marker_separation_m) * pixels_per_meter))
        margin_px = max(10, sep_px)

        inner_w = int(board_width)  * marker_px + max(int(board_width)  - 1, 0) * sep_px
        inner_h = int(board_height) * marker_px + max(int(board_height) - 1, 0) * sep_px
        width_px  = inner_w + 2 * margin_px
        height_px = inner_h + 2 * margin_px

        def _gen(w, h, m):
            if hasattr(board, 'generateImage'):
                return board.generateImage((w, h), m, 1)
            if hasattr(board, 'draw'):
                return board.draw((w, h), m, 1)
            raise RuntimeError("OpenCV ArUco board image generation API unavailable.")

        try:
            img = _gen(width_px, height_px, margin_px)
        except cv2.error:
            # Last-resort fallback: generate into exact inner canvas with no
            # margin, then pad with white border ourselves.
            img = _gen(inner_w, inner_h, 0)
            img = cv2.copyMakeBorder(img, margin_px, margin_px, margin_px, margin_px,
                                     cv2.BORDER_CONSTANT, value=255)

        os.makedirs(os.path.dirname(filepath) or '.', exist_ok=True)
        ok = cv2.imwrite(filepath, img)
        if not ok:
            raise IOError(f"Failed to write ArUco board image: {filepath}")
        return filepath

    def calibrate_intrinsic_aruco(self,
                                  camera_id: str,
                                  images: List[np.ndarray],
                                  dictionary_id: int = ARUCO_DICT_TYPE,
                                  board_width: int = ARUCO_BOARD_WIDTH,
                                  board_height: int = ARUCO_BOARD_HEIGHT,
                                  marker_length_m: float = ARUCO_MARKER_LENGTH_M,
                                  marker_separation_m: float = ARUCO_MARKER_SEPARATION_M,
                                  min_images: int = ARUCO_MIN_IMAGES_INTRINSIC,
                                  min_markers_per_image: int = ARUCO_MIN_MARKERS_PER_IMAGE) -> CameraCalibration:
        """Calibrate camera intrinsics from ArUco board captures."""
        print(f"[StereoCalibration] Calibrating ArUco intrinsics for {camera_id}...")
        dictionary, board = self._build_aruco_board(
            dictionary_id=dictionary_id,
            board_width=board_width,
            board_height=board_height,
            marker_length_m=marker_length_m,
            marker_separation_m=marker_separation_m,
        )

        all_corners = []
        all_ids = []
        marker_counter_per_frame = []
        image_size = None

        for i, img in enumerate(images):
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img
            image_size = gray.shape[::-1]
            corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)

            if ids is None or len(ids) < int(min_markers_per_image):
                print(f"  Insufficient markers in image {i+1}/{len(images)}")
                continue

            all_corners.extend(corners)
            all_ids.extend(ids)
            marker_counter_per_frame.append(len(ids))
            print(f"  Found {len(ids)} markers in image {i+1}/{len(images)}")

        if len(marker_counter_per_frame) < int(min_images):
            raise ValueError(
                f"Not enough valid ArUco images. Found {len(marker_counter_per_frame)}, need at least {min_images}"
            )
        if image_size is None:
            raise ValueError("No valid images provided for ArUco calibration")

        ids_array = np.array(all_ids, dtype=np.int32)
        counter_array = np.array(marker_counter_per_frame, dtype=np.int32)

        rms, camera_matrix, dist_coeffs, _, _ = cv2.aruco.calibrateCameraAruco(
            all_corners,
            ids_array,
            counter_array,
            board,
            image_size,
            None,
            None,
        )

        calibration = CameraCalibration(
            camera_id=camera_id,
            intrinsic_matrix=camera_matrix,
            distortion_coeffs=dist_coeffs,
            image_size=image_size,
            reprojection_error=float(rms)
        )
        self.cameras[camera_id] = calibration
        print(f"  ArUco intrinsic calibration complete. RMS error: {float(rms):.3f} px")
        return calibration
    
    def calibrate_stereo(
        self,
        camera_id_1: str,
        camera_id_2: str,
        images_1: List[np.ndarray],
        images_2: List[np.ndarray],
        checkerboard_size: Tuple[int, int] = (9, 6),
        square_size: float = 0.025
    ) -> Tuple[CameraCalibration, CameraCalibration]:
        """
        Calibrate extrinsic parameters between two cameras.
        
        Args:
            camera_id_1: First camera ID
            camera_id_2: Second camera ID
            images_1: Checkerboard images from camera 1
            images_2: Corresponding checkerboard images from camera 2
            checkerboard_size: Number of internal corners
            square_size: Size of squares in meters
            
        Returns:
            Tuple of calibration objects for both cameras
        """
        print(f"[StereoCalibration] Calibrating stereo pair: {camera_id_1} <-> {camera_id_2}")
        
        # Ensure both cameras have intrinsic calibration
        if camera_id_1 not in self.cameras:
            self.calibrate_intrinsic(camera_id_1, images_1, checkerboard_size, square_size)
        if camera_id_2 not in self.cameras:
            self.calibrate_intrinsic(camera_id_2, images_2, checkerboard_size, square_size)
        
        cal1 = self.cameras[camera_id_1]
        cal2 = self.cameras[camera_id_2]
        
        # Prepare object points
        objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[
            0:checkerboard_size[0],
            0:checkerboard_size[1]
        ].T.reshape(-1, 2)
        objp *= square_size
        
        # Find corners in both image sets
        obj_points = []
        img_points_1 = []
        img_points_2 = []
        
        for i, (img1, img2) in enumerate(zip(images_1, images_2)):
            gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY) if len(img1.shape) == 3 else img1
            gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY) if len(img2.shape) == 3 else img2
            
            ret1, corners1 = cv2.findChessboardCorners(gray1, checkerboard_size, None)
            ret2, corners2 = cv2.findChessboardCorners(gray2, checkerboard_size, None)
            
            if ret1 and ret2:
                obj_points.append(objp)
                
                corners1 = cv2.cornerSubPix(
                    gray1, corners1, (11, 11), (-1, -1), self.calibration_criteria
                )
                corners2 = cv2.cornerSubPix(
                    gray2, corners2, (11, 11), (-1, -1), self.calibration_criteria
                )
                
                img_points_1.append(corners1)
                img_points_2.append(corners2)
                print(f"  Found corners in pair {i+1}/{len(images_1)}")
        
        print(f"  Successfully processed {len(obj_points)} image pairs")
        
        # Stereo calibration
        flags = cv2.CALIB_FIX_INTRINSIC  # Use pre-calibrated intrinsics
        
        ret, _, _, _, _, R, T, E, F = cv2.stereoCalibrate(
            obj_points,
            img_points_1,
            img_points_2,
            cal1.intrinsic_matrix,
            cal1.distortion_coeffs,
            cal2.intrinsic_matrix,
            cal2.distortion_coeffs,
            cal1.image_size,
            criteria=self.calibration_criteria,
            flags=flags
        )
        
        print(f"  Stereo calibration complete. RMS error: {ret:.3f}")

        self.metadata['rms_error'] = float(ret)
        self.metadata['timestamp'] = time.time()
        
        # Set camera 1 as world origin (identity rotation/translation)
        cal1.rotation = np.eye(3)
        cal1.translation = np.zeros((3, 1))
        
        # Camera 2 position relative to camera 1
        cal2.rotation = R
        cal2.translation = T
        
        self.cameras[camera_id_1] = cal1
        self.cameras[camera_id_2] = cal2
        
        return cal1, cal2

    def calibrate_stereo_aruco(self,
                               camera_id_1: str,
                               camera_id_2: str,
                               images_1: List[np.ndarray],
                               images_2: List[np.ndarray],
                               dictionary_id: int = ARUCO_DICT_TYPE,
                               board_width: int = ARUCO_BOARD_WIDTH,
                               board_height: int = ARUCO_BOARD_HEIGHT,
                               marker_length_m: float = ARUCO_MARKER_LENGTH_M,
                               marker_separation_m: float = ARUCO_MARKER_SEPARATION_M,
                               min_pairs: int = ARUCO_MIN_IMAGES_STEREO,
                               min_markers_per_image: int = ARUCO_MIN_MARKERS_PER_IMAGE) -> Tuple[CameraCalibration, CameraCalibration]:
        """Calibrate stereo extrinsics from synchronized ArUco captures."""
        print(f"[StereoCalibration] Calibrating ArUco stereo pair: {camera_id_1} <-> {camera_id_2}")
        dictionary, board = self._build_aruco_board(
            dictionary_id=dictionary_id,
            board_width=board_width,
            board_height=board_height,
            marker_length_m=marker_length_m,
            marker_separation_m=marker_separation_m,
        )

        if camera_id_1 not in self.cameras:
            self.calibrate_intrinsic_aruco(
                camera_id_1, images_1, dictionary_id, board_width, board_height,
                marker_length_m, marker_separation_m
            )
        if camera_id_2 not in self.cameras:
            self.calibrate_intrinsic_aruco(
                camera_id_2, images_2, dictionary_id, board_width, board_height,
                marker_length_m, marker_separation_m
            )

        cal1 = self.cameras[camera_id_1]
        cal2 = self.cameras[camera_id_2]

        board_ids = board.getIds().reshape(-1)
        board_obj_points = board.getObjPoints()
        id_to_obj = {
            int(mid): np.array(board_obj_points[idx], dtype=np.float32)
            for idx, mid in enumerate(board_ids)
        }

        obj_points = []
        img_points_1 = []
        img_points_2 = []
        image_size = None

        for i, (img1, img2) in enumerate(zip(images_1, images_2)):
            gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY) if len(img1.shape) == 3 else img1
            gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY) if len(img2.shape) == 3 else img2
            image_size = gray1.shape[::-1]

            corners1, ids1, _ = cv2.aruco.detectMarkers(gray1, dictionary)
            corners2, ids2, _ = cv2.aruco.detectMarkers(gray2, dictionary)

            if ids1 is None or ids2 is None:
                continue
            if len(ids1) < int(min_markers_per_image) or len(ids2) < int(min_markers_per_image):
                continue

            map1 = {int(iid): c.reshape(4, 2) for iid, c in zip(ids1.reshape(-1), corners1)}
            map2 = {int(iid): c.reshape(4, 2) for iid, c in zip(ids2.reshape(-1), corners2)}
            common_ids = sorted(set(map1.keys()) & set(map2.keys()) & set(id_to_obj.keys()))

            if len(common_ids) < int(min_markers_per_image):
                continue

            obj_list = []
            img1_list = []
            img2_list = []
            for mid in common_ids:
                obj_list.extend(id_to_obj[mid])
                img1_list.extend(map1[mid])
                img2_list.extend(map2[mid])

            obj_arr = np.array(obj_list, dtype=np.float32).reshape(-1, 1, 3)
            i1_arr = np.array(img1_list, dtype=np.float32).reshape(-1, 1, 2)
            i2_arr = np.array(img2_list, dtype=np.float32).reshape(-1, 1, 2)

            obj_points.append(obj_arr)
            img_points_1.append(i1_arr)
            img_points_2.append(i2_arr)
            print(f"  Found {len(common_ids)} common markers in pair {i+1}/{len(images_1)}")

        if len(obj_points) < int(min_pairs):
            raise ValueError(
                f"Not enough valid ArUco stereo pairs. Found {len(obj_points)}, need at least {min_pairs}"
            )
        if image_size is None:
            raise ValueError("No valid stereo image pairs provided")

        flags = cv2.CALIB_FIX_INTRINSIC
        ret, _, _, _, _, R, T, E, F = cv2.stereoCalibrate(
            obj_points,
            img_points_1,
            img_points_2,
            cal1.intrinsic_matrix,
            cal1.distortion_coeffs,
            cal2.intrinsic_matrix,
            cal2.distortion_coeffs,
            image_size,
            criteria=self.calibration_criteria,
            flags=flags,
        )

        self.metadata['rms_error'] = float(ret)
        self.metadata['timestamp'] = time.time()
        self.metadata['calibration_method'] = 'aruco_gridboard'

        cal1.rotation = np.eye(3)
        cal1.translation = np.zeros((3, 1))
        cal2.rotation = R
        cal2.translation = T

        self.cameras[camera_id_1] = cal1
        self.cameras[camera_id_2] = cal2

        print(f"  ArUco stereo calibration complete. RMS error: {float(ret):.3f} px")
        return cal1, cal2
    
    def save_calibration(self, filepath: str):
        """
        Save calibration data to JSON file.
        
        Args:
            filepath: Path to save calibration file
        """
        data = {
            'rms_error': self.metadata.get('rms_error'),
            'timestamp': self.metadata.get('timestamp'),
            'cameras': {}
        }
        
        for camera_id, cal in self.cameras.items():
            data['cameras'][camera_id] = {
                'intrinsic_matrix': cal.intrinsic_matrix.tolist(),
                'distortion_coeffs': cal.distortion_coeffs.tolist(),
                'rotation': cal.rotation.tolist() if cal.rotation is not None else None,
                'translation': cal.translation.tolist() if cal.translation is not None else None,
                'image_size': list(cal.image_size) if cal.image_size else None,
                'reprojection_error': cal.reprojection_error
            }
        
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
        
        print(f"[StereoCalibration] Saved calibration to {filepath}")
    
    def load_calibration(self, filepath: str):
        """
        Load calibration data from JSON file.
        
        Args:
            filepath: Path to calibration file
        """
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"Calibration file not found: {filepath}")
        
        with open(filepath, 'r') as f:
            data = json.load(f)

        # Support both formats:
        # 1) legacy flat format: {cam_id: {...}, cam_id2: {...}}
        # 2) calibration_ui format: {calibration_id, rms_error, ..., cameras: {cam_id: {...}}}
        if isinstance(data, dict) and 'cameras' in data and isinstance(data['cameras'], dict):
            camera_block = data['cameras']
            self.metadata = {
                'calibration_id': data.get('calibration_id'),
                'rms_error': data.get('rms_error'),
                'captures_used': data.get('captures_used'),
                'timestamp': data.get('timestamp')
            }
        else:
            camera_block = data
            self.metadata = {}

        for camera_id, cal_data in camera_block.items():
            if not isinstance(cal_data, dict):
                continue
            if 'intrinsic_matrix' not in cal_data:
                continue

            dist_key = 'distortion_coeffs' if 'distortion_coeffs' in cal_data else 'distortion_coefficients'
            calibration = CameraCalibration(
                camera_id=camera_id,
                intrinsic_matrix=np.array(cal_data['intrinsic_matrix']),
                distortion_coeffs=np.array(cal_data.get(dist_key, [0, 0, 0, 0, 0])),
                rotation=np.array(cal_data['rotation']) if cal_data.get('rotation') is not None else None,
                translation=np.array(cal_data['translation']) if cal_data.get('translation') is not None else None,
                image_size=tuple(cal_data['image_size']) if cal_data.get('image_size') else None,
                reprojection_error=float(cal_data.get('reprojection_error', 0.0) or 0.0)
            )
            self.cameras[camera_id] = calibration
        
        print(f"[StereoCalibration] Loaded calibration for {len(self.cameras)} cameras from {filepath}")
    
    def undistort_point(self, camera_id: str, point: np.ndarray) -> np.ndarray:
        """
        Undistort a 2D image point using camera calibration.
        
        Args:
            camera_id: Camera identifier
            point: 2D point as (x, y) or [[x, y]]
            
        Returns:
            Undistorted point
        """
        if camera_id not in self.cameras:
            raise ValueError(f"No calibration found for camera: {camera_id}")
        
        cal = self.cameras[camera_id]
        
        # Ensure point is in correct format
        if point.ndim == 1:
            point = point.reshape(1, 1, 2)
        elif point.ndim == 2 and point.shape[0] == 1:
            point = point.reshape(1, 1, 2)
        
        undistorted = cv2.undistortPoints(
            point,
            cal.intrinsic_matrix,
            cal.distortion_coeffs,
            P=cal.intrinsic_matrix
        )
        
        return undistorted.reshape(-1, 2)
    
    def get_projection_matrix(self, camera_id: str) -> np.ndarray:
        """
        Get 3x4 projection matrix for a camera.
        P = K [R | t]
        
        Args:
            camera_id: Camera identifier
            
        Returns:
            3x4 projection matrix
        """
        if camera_id not in self.cameras:
            raise ValueError(f"No calibration found for camera: {camera_id}")
        
        cal = self.cameras[camera_id]
        
        if cal.rotation is None or cal.translation is None:
            raise ValueError(f"Extrinsic parameters not calibrated for camera: {camera_id}")
        
        # P = K [R | t]
        RT = np.hstack([cal.rotation, cal.translation])
        P = cal.intrinsic_matrix @ RT
        
        return P

    def create_default_calibration(self, width: int = 1280, height: int = 720) -> None:
        """
        Create a default/approximate calibration for immediate use.
        Assumes two cameras side-by-side, ~1 meter apart.
        """
        print("[StereoCalibration] Creating default calibration (Approximate)")
        
        # Approximate intrinsics for a standard webcam (FOV ~60 deg)
        focal_length = width  # Rough estimate: fx = width
        center_x = width / 2
        center_y = height / 2
        
        K = np.array([
            [focal_length, 0, center_x],
            [0, focal_length, center_y],
            [0, 0, 1]
        ], dtype=np.float32)
        
        dist = np.zeros(5)  # Assume no distortion
        
        # Camera 1 (Origin)
        self.cameras['local_cam'] = CameraCalibration(
            camera_id='local_cam',
            intrinsic_matrix=K,
            distortion_coeffs=dist,
            rotation=np.eye(3),
            translation=np.zeros((3, 1)),
            image_size=(width, height)
        )
        
        # Camera 2 (Remote - PC2)
        # Positioned 1.0 meter to the right of Camera 1
        self.cameras['cam_0'] = CameraCalibration(
            camera_id='cam_0',
            intrinsic_matrix=K,
            distortion_coeffs=dist,
            rotation=np.eye(3),  # Facing same direction
            translation=np.array([[-1.0], [0.0], [0.0]]),  # Translated -1m on X axis (relative to Cam1)
            image_size=(width, height)
        )
        print("[StereoCalibration] Default calibration created (Baseline: 1.0m)")

    def save_intrinsics_yaml(self, camera_id: str, filepath: str):
        """Save per-camera intrinsic calibration to OpenCV YAML."""
        if camera_id not in self.cameras:
            raise ValueError(f"No calibration found for camera: {camera_id}")

        cal = self.cameras[camera_id]
        fs = cv2.FileStorage(filepath, cv2.FILE_STORAGE_WRITE)
        if not fs.isOpened():
            raise IOError(f"Could not open YAML for writing: {filepath}")

        try:
            fs.write("camera_id", camera_id)
            fs.write("K", cal.intrinsic_matrix)
            fs.write("distortion", cal.distortion_coeffs)
            if cal.image_size:
                fs.write("image_width", int(cal.image_size[0]))
                fs.write("image_height", int(cal.image_size[1]))
            fs.write("reprojection_error", float(cal.reprojection_error))
        finally:
            fs.release()

    def save_stereo_yaml(self, camera_id_1: str, camera_id_2: str, filepath: str):
        """Save stereo extrinsics and projection matrices to OpenCV YAML."""
        if camera_id_1 not in self.cameras or camera_id_2 not in self.cameras:
            raise ValueError("Both cameras must be calibrated before exporting stereo YAML")

        cal1 = self.cameras[camera_id_1]
        cal2 = self.cameras[camera_id_2]
        if cal1.rotation is None or cal1.translation is None:
            raise ValueError(f"Extrinsics missing for camera: {camera_id_1}")
        if cal2.rotation is None or cal2.translation is None:
            raise ValueError(f"Extrinsics missing for camera: {camera_id_2}")

        P1 = cal1.intrinsic_matrix @ np.hstack([cal1.rotation, cal1.translation])
        P2 = cal2.intrinsic_matrix @ np.hstack([cal2.rotation, cal2.translation])

        fs = cv2.FileStorage(filepath, cv2.FILE_STORAGE_WRITE)
        if not fs.isOpened():
            raise IOError(f"Could not open YAML for writing: {filepath}")

        try:
            fs.write("camera_1_id", camera_id_1)
            fs.write("camera_2_id", camera_id_2)
            fs.write("R", cal2.rotation)
            fs.write("T", cal2.translation)
            fs.write("P1", P1)
            fs.write("P2", P2)
            baseline = float(np.linalg.norm(cal2.translation - cal1.translation))
            fs.write("baseline_m", baseline)
        finally:
            fs.release()

    def check_reprojection_error_threshold(self, camera_id: str, threshold_px: float = 1.0) -> Dict[str, Any]:
        """Check whether a camera's reprojection error meets the desired threshold."""
        if camera_id not in self.cameras:
            raise ValueError(f"No calibration found for camera: {camera_id}")

        error_px = float(self.cameras[camera_id].reprojection_error)
        passed = bool(error_px < float(threshold_px))
        return {
            'camera_id': camera_id,
            'reprojection_error_px': error_px,
            'threshold_px': float(threshold_px),
            'pass': passed,
            'status': 'EXCELLENT' if error_px < 1.0 else ('ACCEPTABLE' if error_px < 2.0 else 'POOR'),
        }


if __name__ == "__main__":
    print("Stereo Calibration Module - Test")
    cal = StereoCalibration()
    cal.create_default_calibration()
