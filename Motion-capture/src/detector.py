import mediapipe as mp
import cv2
import time
import numpy as np
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from config import (
    MODEL_PATHS, POSE_MODEL_COMPLEXITY, NUM_POSES, NUM_FACES, NUM_HANDS,
    MIN_DETECTION_CONFIDENCE, MIN_TRACKING_CONFIDENCE,
    MAX_FRAME_WIDTH, FRAME_WIDTH, GAMMA_DEFAULT, GAMMA_MIN, GAMMA_MAX,
    FACE_EXPOSURE_TARGET, FACE_EXPOSURE_MIN_GAIN, FACE_EXPOSURE_MAX_GAIN,
    FACE_EXPOSURE_MIN_BRIGHTNESS, CLAHE_CLIP_LIMIT, CLAHE_TILE_GRID_SIZE,
    ENABLE_FACE_DETECTION, ENABLE_HAND_DETECTION, ENABLE_FACE_EXPOSURE,
    ENABLE_ROI_CROPPING, ROI_EXPANSION_FACTOR, ROI_MIN_SIZE,
    ROI_TARGET_SIZE, ROI_SMOOTHING_ALPHA, PREFER_GPU_DELEGATE,
    INFERENCE_BACKEND,
    ABLATION_DISABLE_FRAME_RESIZE, ABLATION_DISABLE_CLAHE_DEL,
    CUDA_ENABLED, MPS_ENABLED, MULTI_CAMERA_MODE
)

class MocapDetector:
    def __init__(self):
        # Multi-camera runtime only needs pose landmarks for sync/triangulation.
        # Keep face/hand off by default there to avoid loading extra MediaPipe models.
        self.enable_face = ENABLE_FACE_DETECTION if MULTI_CAMERA_MODE == 'single' else False
        self.enable_hand = ENABLE_HAND_DETECTION if MULTI_CAMERA_MODE == 'single' else False
        
        # Imaging Settings
        self.gamma = GAMMA_DEFAULT
        self.face_exposure = ENABLE_FACE_EXPOSURE
        self.last_face_rect = None # (x, y, w, h)
        self.last_timestamp_ms = 0
        
        # ROI Cropping
        self.enable_roi_cropping = ENABLE_ROI_CROPPING
        self.last_roi = None  # (x, y, w, h) in original frame coords
        self.roi_expansion = ROI_EXPANSION_FACTOR
        self.roi_min_size = ROI_MIN_SIZE
        self.roi_target_size = ROI_TARGET_SIZE
        self.roi_alpha = ROI_SMOOTHING_ALPHA

        # Pre-allocate CLAHE once (reused every frame to avoid per-frame allocation leak)
        self._clahe = cv2.createCLAHE(clipLimit=CLAHE_CLIP_LIMIT, tileGridSize=CLAHE_TILE_GRID_SIZE)
        
        # Select Pose Model based on config
        pose_model_path = MODEL_PATHS.get(POSE_MODEL_COMPLEXITY, MODEL_PATHS['LITE'])
        print(f"Loading Pose Model: {POSE_MODEL_COMPLEXITY} ({pose_model_path})")

        self.use_gpu_delegate = self._should_use_gpu_delegate()
        if self.use_gpu_delegate:
            print("[Detector] Using MediaPipe GPU delegate (Metal on macOS)")
        else:
            print("[Detector] Using CPU delegate")

        # 1. Pose Landmarker
        self.pose_landmarker = self._build_pose_landmarker(pose_model_path)

        # 2/3. Face+Hand are loaded lazily only when enabled.
        self.face_landmarker = None
        self.hand_landmarker = None
        if self.enable_face:
            self.face_landmarker = self._build_face_landmarker(MODEL_PATHS['FACE'])
        if self.enable_hand:
            self.hand_landmarker = self._build_hand_landmarker(MODEL_PATHS['HAND'])

        # GPU delegate flag for hardware-specific frame handling:
        # CVPixelBuffer on Apple Silicon requires height to be a multiple of 16.
        # Clamp to actual runtime GPU availability so the flag is never True
        # when no accelerator is present (prevents -6662 kCVReturnInvalidSize).
        self.use_gpu_delegate = self.use_gpu_delegate and (CUDA_ENABLED or MPS_ENABLED)

    def _ensure_gpu_safe_size(self, frame):
        """Ensure dimensions are CVPixelBuffer-safe for MediaPipe GPU/Metal paths.
        On macOS Metal we have observed fatal -6662 errors when frame dims are not aligned.
        """
        if not self.use_gpu_delegate or frame is None:
            return frame
        h, w = frame.shape[:2]
        # Keep dimensions aligned for stable CVPixelBuffer allocation.
        safe_w = max(16, ((w + 15) // 16) * 16)
        safe_h = max(16, ((h + 15) // 16) * 16)
        if safe_w != w or safe_h != h:
            frame = cv2.resize(frame, (safe_w, safe_h), interpolation=cv2.INTER_AREA)
        return frame

    def _should_use_gpu_delegate(self):
        backend = str(INFERENCE_BACKEND).strip().lower()

        if backend == 'cpu':
            return False

        if backend == 'mps':
            if MPS_ENABLED:
                return True
            print("[Detector] INFERENCE_BACKEND='mps' requested but MPS is unavailable. Falling back to CPU")
            return False

        if backend == 'gpu':
            return CUDA_ENABLED or MPS_ENABLED

        if backend == 'auto':
            return bool(PREFER_GPU_DELEGATE and (CUDA_ENABLED or MPS_ENABLED))

        print(f"[Detector] Unknown INFERENCE_BACKEND='{INFERENCE_BACKEND}'. Using auto mode")
        return bool(PREFER_GPU_DELEGATE and (CUDA_ENABLED or MPS_ENABLED))

    def _create_base_options(self, model_path):
        # Force CPU delegate path to avoid long-run Metal CVPixelBuffer leaks on macOS.
        self.use_gpu_delegate = False
        return python.BaseOptions(model_asset_path=model_path)

    def _build_pose_landmarker(self, model_path):
        try:
            base_opts = self._create_base_options(model_path)
            pose_opts = vision.PoseLandmarkerOptions(
                base_options=base_opts,
                running_mode=vision.RunningMode.VIDEO,
                num_poses=NUM_POSES,
                min_pose_detection_confidence=MIN_DETECTION_CONFIDENCE,
                min_pose_presence_confidence=MIN_TRACKING_CONFIDENCE,
                min_tracking_confidence=MIN_TRACKING_CONFIDENCE
            )
            return vision.PoseLandmarker.create_from_options(pose_opts)
        except Exception as error:
            if self.use_gpu_delegate:
                print(f"[Detector] Pose GPU init failed ({error}), retrying on CPU")
                self.use_gpu_delegate = False
                return self._build_pose_landmarker(model_path)
            raise

    def _build_face_landmarker(self, model_path):
        try:
            base_opts = self._create_base_options(model_path)
            face_opts = vision.FaceLandmarkerOptions(
                base_options=base_opts,
                running_mode=vision.RunningMode.VIDEO,
                num_faces=NUM_FACES,
                min_face_detection_confidence=MIN_DETECTION_CONFIDENCE,
                min_face_presence_confidence=MIN_TRACKING_CONFIDENCE,
                min_tracking_confidence=MIN_TRACKING_CONFIDENCE
            )
            return vision.FaceLandmarker.create_from_options(face_opts)
        except Exception as error:
            if self.use_gpu_delegate:
                print(f"[Detector] Face GPU init failed ({error}), retrying on CPU")
                self.use_gpu_delegate = False
                return self._build_face_landmarker(model_path)
            raise

    def _build_hand_landmarker(self, model_path):
        try:
            base_opts = self._create_base_options(model_path)
            hand_opts = vision.HandLandmarkerOptions(
                base_options=base_opts,
                running_mode=vision.RunningMode.VIDEO,
                num_hands=NUM_HANDS,
                min_hand_detection_confidence=MIN_DETECTION_CONFIDENCE,
                min_hand_presence_confidence=MIN_TRACKING_CONFIDENCE,
                min_tracking_confidence=MIN_TRACKING_CONFIDENCE
            )
            return vision.HandLandmarker.create_from_options(hand_opts)
        except Exception as error:
            if self.use_gpu_delegate:
                print(f"[Detector] Hand GPU init failed ({error}), retrying on CPU")
                self.use_gpu_delegate = False
                return self._build_hand_landmarker(model_path)
            raise
        
    def reload(self, model_type="FULL"):
        """Reload Pose Model with specific complexity (LITE, FULL, HEAVY)."""
        if model_type not in MODEL_PATHS:
            print(f"Unknown model type: {model_type}")
            return

        print(f"Reloading Pose Model: {model_type}")
        if hasattr(self, 'pose_landmarker'):
            self.pose_landmarker.close()
            
        pose_model_path = MODEL_PATHS[model_type]
        self.pose_landmarker = self._build_pose_landmarker(pose_model_path)

    def get_gpu_profile(self) -> dict:
        """Get GPU inference timing statistics."""
        if not hasattr(self, '_gpu_profile_accum') or not self._gpu_profile_accum.get('pose'):
            return {}
        acc = self._gpu_profile_accum
        n = min(len(acc['pose']), 100)
        return {
            'pose_ms': sum(acc['pose'][-n:]) / n,
            'face_ms': sum(acc['face'][-n:]) / n if acc['face'] else 0,
            'hand_ms': sum(acc['hand'][-n:]) / n if acc['hand'] else 0,
            'delegate': 'Metal' if self.use_gpu_delegate else 'CPU',
            'total_frames': acc['count']
        }
        
    def set_imaging_params(self, gamma=1.0, face_exposure=False, enable_face=True, enable_hand=True, enable_roi=None):
        """Update runtime imaging parameters."""
        self.gamma = gamma
        self.face_exposure = face_exposure
        self.enable_face = bool(enable_face)
        self.enable_hand = bool(enable_hand)

        # Lazy-init optional models only when toggled on.
        if self.enable_face and self.face_landmarker is None:
            self.face_landmarker = self._build_face_landmarker(MODEL_PATHS['FACE'])
        if self.enable_hand and self.hand_landmarker is None:
            self.hand_landmarker = self._build_hand_landmarker(MODEL_PATHS['HAND'])

        if enable_roi is not None:
            self.enable_roi_cropping = enable_roi

    def _apply_gamma(self, image, gamma=1.0):
        if gamma == 1.0: return image
        invGamma = 1.0 / gamma
        table = np.array([((i / 255.0) ** invGamma) * 255 for i in np.arange(0, 256)]).astype("uint8")
        return cv2.LUT(image, table)

    def _apply_face_exposure(self, image):
        """
        Normalize brightness based on previous face detection.
        Target brightness from config.
        """
        if self.last_face_rect is None: return image
        
        x, y, w, h = self.last_face_rect
        # Valid bounds check
        H, W = image.shape[:2]
        x = max(0, min(x, W-1))
        y = max(0, min(y, H-1))
        w = max(1, min(w, W-x))
        h = max(1, min(h, H-y))
        
        roi = image[y:y+h, x:x+w]
        if roi.size == 0: return image
        
        # Calculate mean V (Brightness)
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        v_mean = np.mean(hsv_roi[:,:,2])
        
        if v_mean < FACE_EXPOSURE_MIN_BRIGHTNESS:
            v_mean = FACE_EXPOSURE_MIN_BRIGHTNESS
        
        factor = FACE_EXPOSURE_TARGET / v_mean
        factor = min(factor, FACE_EXPOSURE_MAX_GAIN)
        factor = max(factor, FACE_EXPOSURE_MIN_GAIN)
        
        # Apply gain
        return cv2.convertScaleAbs(image, alpha=factor, beta=0)

    def process(self, frame):
        """
        Process a frame and return pose, face, and hand results.
        Expects BGR frame.
        """
        import numpy as np
        
        H_orig, W_orig = frame.shape[:2]
        
        # --- ROI CROPPING (High Quality) ---
        # We crop from the ORIGINAL frame to preserve maximum detail.
        roi_x, roi_y, roi_w, roi_h = 0, 0, W_orig, H_orig # Defaults
        roi_active = False
        
        # Determine effective frame to process
        processing_frame = frame
        
        if self.enable_roi_cropping and self.last_roi is not None:
            x, y, w, h = self.last_roi
            
            # ROI Coords are in W_orig, H_orig scale
            # Expand ROI
            expand = int(max(w, h) * self.roi_expansion)
            x = max(0, x - expand)
            y = max(0, y - expand)
            w = min(W_orig - x, w + 2 * expand)
            h = min(H_orig - y, h + 2 * expand)
            
            # Ensure minimum size
            if w >= self.roi_min_size and h >= self.roi_min_size:
                # Crop from original (unprocessed) frame
                roi_frame = frame[y:y+h, x:x+w]
                
                # Resize to target size for inference
                # e.g. Crop 500x500 -> Resize 640x640 (Upscale or Downscale)
                max_dim = max(w, h)
                scale_to_target = self.roi_target_size / max_dim
                
                new_w = int(w * scale_to_target)
                new_h = int(h * scale_to_target)
                processing_frame = cv2.resize(roi_frame, (new_w, new_h))
                
                # Store EXACT ROI params for reprojection formula
                roi_x, roi_y = x, y
                roi_w, roi_h = w, h
                roi_active = True
            else:
                # ROI invalid/too small? Fallback to full frame
                processing_frame = frame
                # Standard Resize if full frame is massive
                if not ABLATION_DISABLE_FRAME_RESIZE and W_orig > MAX_FRAME_WIDTH:
                   scale = MAX_FRAME_WIDTH / W_orig
                   new_h = int(H_orig * scale)
                   # CVPixelBuffer (Metal/MPS) requires height to be a multiple of 16.
                   # int() truncation can produce e.g. 360 (not /16), causing -6662 abort.
                   if self.use_gpu_delegate:
                       new_h = ((new_h + 15) // 16) * 16
                   processing_frame = cv2.resize(frame, (MAX_FRAME_WIDTH, new_h))
        else:
            # 1. Resize for consistency (if too large)
            if not ABLATION_DISABLE_FRAME_RESIZE and W_orig > MAX_FRAME_WIDTH:
                scale = MAX_FRAME_WIDTH / W_orig
                new_h = int(H_orig * scale)
                # CVPixelBuffer (Metal/MPS) requires height to be a multiple of 16.
                # int() truncation can produce e.g. 360 (not /16), causing -6662 abort.
                if self.use_gpu_delegate:
                    new_h = ((new_h + 15) // 16) * 16
                processing_frame = cv2.resize(frame, (MAX_FRAME_WIDTH, new_h))
                
        # Update frame reference for filters
        frame = processing_frame

        # Final guard: all GPU paths (including ROI crops) must be CVPixelBuffer-safe.
        frame = self._ensure_gpu_safe_size(frame)
            
        # 2. Gamma Correction
        if self.gamma != 1.0:
            frame = self._apply_gamma(frame, self.gamma)
            
        # 3. Face-Guided Exposure
        if self.face_exposure:
            frame = self._apply_face_exposure(frame)
            
        # 4. CLAHE (reuse pre-allocated instance — avoids per-frame allocation)
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        cl = self._clahe.apply(l)
        limg = cv2.merge((cl, a, b))
        frame = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)
        if not ABLATION_DISABLE_CLAHE_DEL:
            del lab, l, a, b, cl, limg  # free intermediates immediately (640x360x3 each)
        # ---------------------
        
        # Convert frame for MediaPipe Image
        if self.use_gpu_delegate:
            # GPU path on macOS is more stable with SRGBA input
            frame_rgba = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGBA, data=frame_rgba)
            del frame_rgba  # mp.Image has copied data into Metal buffer; free Python copy
        else:
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame_rgb)
            del frame_rgb   # mp.Image has its own copy; free Python copy
        
        # Calculate Timestamp in Milliseconds
        timestamp_ms = int(time.time() * 1000)
        
        # Monotonicity Check
        if timestamp_ms <= self.last_timestamp_ms:
            timestamp_ms = self.last_timestamp_ms + 1
        self.last_timestamp_ms = timestamp_ms
        
        # Run Detectors (with GPU profiling)
        from config import ENABLE_GPU_PROFILING
        
        t_pose_start = time.perf_counter()
        pose_result = self.pose_landmarker.detect_for_video(mp_image, timestamp_ms)
        t_pose_end = time.perf_counter()
        
        # --- REPROJECT LANDMARKS TO ORIGINAL FRAME ---
        if pose_result and pose_result.pose_landmarks:
            plm = pose_result.pose_landmarks[0]
            
            # Reproject landmarks in-place using User's Formula
            if roi_active:
                for lm in plm:
                    # User Formula: full_x = roi_x + lm.x * roi_w
                    px_full_x = roi_x + lm.x * roi_w
                    px_full_y = roi_y + lm.y * roi_h
                    
                    # Normalize back to [0,1] of ORIGINAL frame
                    lm.x = px_full_x / W_orig
                    lm.y = px_full_y / H_orig
            else:
                 # Standard logic (if simple resize happens)
                 # If we resized the input frame, we just need to confirm aspect ratio.
                 # MP outputs normalized [0,1].
                 # If we resized 1920x1080 -> 960x540, normalized x=0.5 is still x=0.5.
                 # So NO REPROJECTION needed for standard resize 
                 # UNLESS aspect ratio changed (which we avoided).
                 pass
            
            # Now lm.x, lm.y are normalized to ORIGINAL frame.
            
            # --- UPDATE NEXT ROI ---
            xs = [lm.x * W_orig for lm in plm]
            ys = [lm.y * H_orig for lm in plm]
            
            x_min, x_max = int(min(xs)), int(max(xs))
            y_min, y_max = int(min(ys)), int(max(ys))
            new_roi = (x_min, y_min, x_max - x_min, y_max - y_min)
            
            # Smooth ROI position
            if self.enable_roi_cropping:
                if self.last_roi is not None:
                    a = self.roi_alpha
                    self.last_roi = (
                        int(a * new_roi[0] + (1-a) * self.last_roi[0]),
                        int(a * new_roi[1] + (1-a) * self.last_roi[1]),
                        int(a * new_roi[2] + (1-a) * self.last_roi[2]),
                        int(a * new_roi[3] + (1-a) * self.last_roi[3])
                    )
                else:
                    self.last_roi = new_roi
            else:
                 self.last_roi = None

        elif self.enable_roi_cropping:
             # Lost tracking? Reset.
             self.last_roi = None 

        
        face_result = None
        t_face_ms = 0
        if self.enable_face and self.face_landmarker is not None:
            t_face_start = time.perf_counter()
            face_result = self.face_landmarker.detect_for_video(mp_image, timestamp_ms)
            t_face_ms = (time.perf_counter() - t_face_start) * 1000
            # Update Face Rect for Exposure (from first face)
            if face_result and face_result.face_landmarks:
                 # Calculate bounding box
                 flm = face_result.face_landmarks[0]
                 xs = [lm.x for lm in flm]
                 ys = [lm.y for lm in flm]
                 H, W = H_orig, W_orig
                 x_min, x_max = min(xs) * W, max(xs) * W
                 y_min, y_max = min(ys) * H, max(ys) * H
                 self.last_face_rect = (int(x_min), int(y_min), int(x_max-x_min), int(y_max-y_min))
        
        hand_result = None
        t_hand_ms = 0
        if self.enable_hand and self.hand_landmarker is not None:
            t_hand_start = time.perf_counter()
            hand_result = self.hand_landmarker.detect_for_video(mp_image, timestamp_ms)
            t_hand_ms = (time.perf_counter() - t_hand_start) * 1000
        
        # GPU profiling accumulator
        t_pose_ms = (t_pose_end - t_pose_start) * 1000
        if ENABLE_GPU_PROFILING:
            if not hasattr(self, '_gpu_profile_accum'):
                self._gpu_profile_accum = {'pose': [], 'face': [], 'hand': [], 'count': 0}
            self._gpu_profile_accum['pose'].append(t_pose_ms)
            self._gpu_profile_accum['face'].append(t_face_ms)
            self._gpu_profile_accum['hand'].append(t_hand_ms)
            self._gpu_profile_accum['count'] += 1

            max_samples = 600
            if len(self._gpu_profile_accum['pose']) > max_samples:
                self._gpu_profile_accum['pose'] = self._gpu_profile_accum['pose'][-300:]
            if len(self._gpu_profile_accum['face']) > max_samples:
                self._gpu_profile_accum['face'] = self._gpu_profile_accum['face'][-300:]
            if len(self._gpu_profile_accum['hand']) > max_samples:
                self._gpu_profile_accum['hand'] = self._gpu_profile_accum['hand'][-300:]
            
            # Log every 300 frames
            if self._gpu_profile_accum['count'] % 300 == 0:
                n = len(self._gpu_profile_accum['pose'])
                avg_p = sum(self._gpu_profile_accum['pose'][-100:]) / min(n, 100)
                avg_f = sum(self._gpu_profile_accum['face'][-100:]) / min(n, 100) if self._gpu_profile_accum['face'] else 0
                avg_h = sum(self._gpu_profile_accum['hand'][-100:]) / min(n, 100) if self._gpu_profile_accum['hand'] else 0
                total = avg_p + avg_f + avg_h
                print(f"[GPU Profile] Pose: {avg_p:.1f}ms  Face: {avg_f:.1f}ms  "
                      f"Hand: {avg_h:.1f}ms  Total: {total:.1f}ms  "
                      f"(GPU: {'Metal' if self.use_gpu_delegate else 'CPU'})")
        
        return {
            'pose': pose_result,
            'face': face_result,
            'hand': hand_result,
            'roi': self.last_roi if self.enable_roi_cropping else None,
            'inference_ms': {
                'pose': t_pose_ms,
                'face': t_face_ms,
                'hand': t_hand_ms
            }
        }


